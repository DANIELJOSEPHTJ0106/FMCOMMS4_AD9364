// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Wed Jan  7 14:22:25 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ fifo_generator_3_sim_netlist.v
// Design      : fifo_generator_3
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CHECK_LICENSE_TYPE = "fifo_generator_3,fifo_generator_v13_2_8,{}" *) (* downgradeipidentifiedwarnings = "yes" *) (* x_core_info = "fifo_generator_v13_2_8,Vivado 2023.1" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (clk,
    srst,
    din,
    wr_en,
    rd_en,
    dout,
    full,
    empty);
  (* x_interface_info = "xilinx.com:signal:clock:1.0 core_clk CLK" *) (* x_interface_parameter = "XIL_INTERFACENAME core_clk, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, INSERT_VIP 0" *) input clk;
  input srst;
  (* x_interface_info = "xilinx.com:interface:fifo_write:1.0 FIFO_WRITE WR_DATA" *) input [0:0]din;
  (* x_interface_info = "xilinx.com:interface:fifo_write:1.0 FIFO_WRITE WR_EN" *) input wr_en;
  (* x_interface_info = "xilinx.com:interface:fifo_read:1.0 FIFO_READ RD_EN" *) input rd_en;
  (* x_interface_info = "xilinx.com:interface:fifo_read:1.0 FIFO_READ RD_DATA" *) output [0:0]dout;
  (* x_interface_info = "xilinx.com:interface:fifo_write:1.0 FIFO_WRITE FULL" *) output full;
  (* x_interface_info = "xilinx.com:interface:fifo_read:1.0 FIFO_READ EMPTY" *) output empty;

  wire clk;
  wire [0:0]din;
  wire [0:0]dout;
  wire empty;
  wire full;
  wire rd_en;
  wire srst;
  wire wr_en;
  wire NLW_U0_almost_empty_UNCONNECTED;
  wire NLW_U0_almost_full_UNCONNECTED;
  wire NLW_U0_axi_ar_dbiterr_UNCONNECTED;
  wire NLW_U0_axi_ar_overflow_UNCONNECTED;
  wire NLW_U0_axi_ar_prog_empty_UNCONNECTED;
  wire NLW_U0_axi_ar_prog_full_UNCONNECTED;
  wire NLW_U0_axi_ar_sbiterr_UNCONNECTED;
  wire NLW_U0_axi_ar_underflow_UNCONNECTED;
  wire NLW_U0_axi_aw_dbiterr_UNCONNECTED;
  wire NLW_U0_axi_aw_overflow_UNCONNECTED;
  wire NLW_U0_axi_aw_prog_empty_UNCONNECTED;
  wire NLW_U0_axi_aw_prog_full_UNCONNECTED;
  wire NLW_U0_axi_aw_sbiterr_UNCONNECTED;
  wire NLW_U0_axi_aw_underflow_UNCONNECTED;
  wire NLW_U0_axi_b_dbiterr_UNCONNECTED;
  wire NLW_U0_axi_b_overflow_UNCONNECTED;
  wire NLW_U0_axi_b_prog_empty_UNCONNECTED;
  wire NLW_U0_axi_b_prog_full_UNCONNECTED;
  wire NLW_U0_axi_b_sbiterr_UNCONNECTED;
  wire NLW_U0_axi_b_underflow_UNCONNECTED;
  wire NLW_U0_axi_r_dbiterr_UNCONNECTED;
  wire NLW_U0_axi_r_overflow_UNCONNECTED;
  wire NLW_U0_axi_r_prog_empty_UNCONNECTED;
  wire NLW_U0_axi_r_prog_full_UNCONNECTED;
  wire NLW_U0_axi_r_sbiterr_UNCONNECTED;
  wire NLW_U0_axi_r_underflow_UNCONNECTED;
  wire NLW_U0_axi_w_dbiterr_UNCONNECTED;
  wire NLW_U0_axi_w_overflow_UNCONNECTED;
  wire NLW_U0_axi_w_prog_empty_UNCONNECTED;
  wire NLW_U0_axi_w_prog_full_UNCONNECTED;
  wire NLW_U0_axi_w_sbiterr_UNCONNECTED;
  wire NLW_U0_axi_w_underflow_UNCONNECTED;
  wire NLW_U0_axis_dbiterr_UNCONNECTED;
  wire NLW_U0_axis_overflow_UNCONNECTED;
  wire NLW_U0_axis_prog_empty_UNCONNECTED;
  wire NLW_U0_axis_prog_full_UNCONNECTED;
  wire NLW_U0_axis_sbiterr_UNCONNECTED;
  wire NLW_U0_axis_underflow_UNCONNECTED;
  wire NLW_U0_dbiterr_UNCONNECTED;
  wire NLW_U0_m_axi_arvalid_UNCONNECTED;
  wire NLW_U0_m_axi_awvalid_UNCONNECTED;
  wire NLW_U0_m_axi_bready_UNCONNECTED;
  wire NLW_U0_m_axi_rready_UNCONNECTED;
  wire NLW_U0_m_axi_wlast_UNCONNECTED;
  wire NLW_U0_m_axi_wvalid_UNCONNECTED;
  wire NLW_U0_m_axis_tlast_UNCONNECTED;
  wire NLW_U0_m_axis_tvalid_UNCONNECTED;
  wire NLW_U0_overflow_UNCONNECTED;
  wire NLW_U0_prog_empty_UNCONNECTED;
  wire NLW_U0_prog_full_UNCONNECTED;
  wire NLW_U0_rd_rst_busy_UNCONNECTED;
  wire NLW_U0_s_axi_arready_UNCONNECTED;
  wire NLW_U0_s_axi_awready_UNCONNECTED;
  wire NLW_U0_s_axi_bvalid_UNCONNECTED;
  wire NLW_U0_s_axi_rlast_UNCONNECTED;
  wire NLW_U0_s_axi_rvalid_UNCONNECTED;
  wire NLW_U0_s_axi_wready_UNCONNECTED;
  wire NLW_U0_s_axis_tready_UNCONNECTED;
  wire NLW_U0_sbiterr_UNCONNECTED;
  wire NLW_U0_underflow_UNCONNECTED;
  wire NLW_U0_valid_UNCONNECTED;
  wire NLW_U0_wr_ack_UNCONNECTED;
  wire NLW_U0_wr_rst_busy_UNCONNECTED;
  wire [4:0]NLW_U0_axi_ar_data_count_UNCONNECTED;
  wire [4:0]NLW_U0_axi_ar_rd_data_count_UNCONNECTED;
  wire [4:0]NLW_U0_axi_ar_wr_data_count_UNCONNECTED;
  wire [4:0]NLW_U0_axi_aw_data_count_UNCONNECTED;
  wire [4:0]NLW_U0_axi_aw_rd_data_count_UNCONNECTED;
  wire [4:0]NLW_U0_axi_aw_wr_data_count_UNCONNECTED;
  wire [4:0]NLW_U0_axi_b_data_count_UNCONNECTED;
  wire [4:0]NLW_U0_axi_b_rd_data_count_UNCONNECTED;
  wire [4:0]NLW_U0_axi_b_wr_data_count_UNCONNECTED;
  wire [10:0]NLW_U0_axi_r_data_count_UNCONNECTED;
  wire [10:0]NLW_U0_axi_r_rd_data_count_UNCONNECTED;
  wire [10:0]NLW_U0_axi_r_wr_data_count_UNCONNECTED;
  wire [10:0]NLW_U0_axi_w_data_count_UNCONNECTED;
  wire [10:0]NLW_U0_axi_w_rd_data_count_UNCONNECTED;
  wire [10:0]NLW_U0_axi_w_wr_data_count_UNCONNECTED;
  wire [10:0]NLW_U0_axis_data_count_UNCONNECTED;
  wire [10:0]NLW_U0_axis_rd_data_count_UNCONNECTED;
  wire [10:0]NLW_U0_axis_wr_data_count_UNCONNECTED;
  wire [11:0]NLW_U0_data_count_UNCONNECTED;
  wire [31:0]NLW_U0_m_axi_araddr_UNCONNECTED;
  wire [1:0]NLW_U0_m_axi_arburst_UNCONNECTED;
  wire [3:0]NLW_U0_m_axi_arcache_UNCONNECTED;
  wire [0:0]NLW_U0_m_axi_arid_UNCONNECTED;
  wire [7:0]NLW_U0_m_axi_arlen_UNCONNECTED;
  wire [0:0]NLW_U0_m_axi_arlock_UNCONNECTED;
  wire [2:0]NLW_U0_m_axi_arprot_UNCONNECTED;
  wire [3:0]NLW_U0_m_axi_arqos_UNCONNECTED;
  wire [3:0]NLW_U0_m_axi_arregion_UNCONNECTED;
  wire [2:0]NLW_U0_m_axi_arsize_UNCONNECTED;
  wire [0:0]NLW_U0_m_axi_aruser_UNCONNECTED;
  wire [31:0]NLW_U0_m_axi_awaddr_UNCONNECTED;
  wire [1:0]NLW_U0_m_axi_awburst_UNCONNECTED;
  wire [3:0]NLW_U0_m_axi_awcache_UNCONNECTED;
  wire [0:0]NLW_U0_m_axi_awid_UNCONNECTED;
  wire [7:0]NLW_U0_m_axi_awlen_UNCONNECTED;
  wire [0:0]NLW_U0_m_axi_awlock_UNCONNECTED;
  wire [2:0]NLW_U0_m_axi_awprot_UNCONNECTED;
  wire [3:0]NLW_U0_m_axi_awqos_UNCONNECTED;
  wire [3:0]NLW_U0_m_axi_awregion_UNCONNECTED;
  wire [2:0]NLW_U0_m_axi_awsize_UNCONNECTED;
  wire [0:0]NLW_U0_m_axi_awuser_UNCONNECTED;
  wire [63:0]NLW_U0_m_axi_wdata_UNCONNECTED;
  wire [0:0]NLW_U0_m_axi_wid_UNCONNECTED;
  wire [7:0]NLW_U0_m_axi_wstrb_UNCONNECTED;
  wire [0:0]NLW_U0_m_axi_wuser_UNCONNECTED;
  wire [7:0]NLW_U0_m_axis_tdata_UNCONNECTED;
  wire [0:0]NLW_U0_m_axis_tdest_UNCONNECTED;
  wire [0:0]NLW_U0_m_axis_tid_UNCONNECTED;
  wire [0:0]NLW_U0_m_axis_tkeep_UNCONNECTED;
  wire [0:0]NLW_U0_m_axis_tstrb_UNCONNECTED;
  wire [3:0]NLW_U0_m_axis_tuser_UNCONNECTED;
  wire [11:0]NLW_U0_rd_data_count_UNCONNECTED;
  wire [0:0]NLW_U0_s_axi_bid_UNCONNECTED;
  wire [1:0]NLW_U0_s_axi_bresp_UNCONNECTED;
  wire [0:0]NLW_U0_s_axi_buser_UNCONNECTED;
  wire [63:0]NLW_U0_s_axi_rdata_UNCONNECTED;
  wire [0:0]NLW_U0_s_axi_rid_UNCONNECTED;
  wire [1:0]NLW_U0_s_axi_rresp_UNCONNECTED;
  wire [0:0]NLW_U0_s_axi_ruser_UNCONNECTED;
  wire [11:0]NLW_U0_wr_data_count_UNCONNECTED;

  (* C_ADD_NGC_CONSTRAINT = "0" *) 
  (* C_APPLICATION_TYPE_AXIS = "0" *) 
  (* C_APPLICATION_TYPE_RACH = "0" *) 
  (* C_APPLICATION_TYPE_RDCH = "0" *) 
  (* C_APPLICATION_TYPE_WACH = "0" *) 
  (* C_APPLICATION_TYPE_WDCH = "0" *) 
  (* C_APPLICATION_TYPE_WRCH = "0" *) 
  (* C_AXIS_TDATA_WIDTH = "8" *) 
  (* C_AXIS_TDEST_WIDTH = "1" *) 
  (* C_AXIS_TID_WIDTH = "1" *) 
  (* C_AXIS_TKEEP_WIDTH = "1" *) 
  (* C_AXIS_TSTRB_WIDTH = "1" *) 
  (* C_AXIS_TUSER_WIDTH = "4" *) 
  (* C_AXIS_TYPE = "0" *) 
  (* C_AXI_ADDR_WIDTH = "32" *) 
  (* C_AXI_ARUSER_WIDTH = "1" *) 
  (* C_AXI_AWUSER_WIDTH = "1" *) 
  (* C_AXI_BUSER_WIDTH = "1" *) 
  (* C_AXI_DATA_WIDTH = "64" *) 
  (* C_AXI_ID_WIDTH = "1" *) 
  (* C_AXI_LEN_WIDTH = "8" *) 
  (* C_AXI_LOCK_WIDTH = "1" *) 
  (* C_AXI_RUSER_WIDTH = "1" *) 
  (* C_AXI_TYPE = "1" *) 
  (* C_AXI_WUSER_WIDTH = "1" *) 
  (* C_COMMON_CLOCK = "1" *) 
  (* C_COUNT_TYPE = "0" *) 
  (* C_DATA_COUNT_WIDTH = "12" *) 
  (* C_DEFAULT_VALUE = "BlankString" *) 
  (* C_DIN_WIDTH = "1" *) 
  (* C_DIN_WIDTH_AXIS = "1" *) 
  (* C_DIN_WIDTH_RACH = "32" *) 
  (* C_DIN_WIDTH_RDCH = "64" *) 
  (* C_DIN_WIDTH_WACH = "1" *) 
  (* C_DIN_WIDTH_WDCH = "64" *) 
  (* C_DIN_WIDTH_WRCH = "2" *) 
  (* C_DOUT_RST_VAL = "0" *) 
  (* C_DOUT_WIDTH = "1" *) 
  (* C_ENABLE_RLOCS = "0" *) 
  (* C_ENABLE_RST_SYNC = "1" *) 
  (* C_EN_SAFETY_CKT = "0" *) 
  (* C_ERROR_INJECTION_TYPE = "0" *) 
  (* C_ERROR_INJECTION_TYPE_AXIS = "0" *) 
  (* C_ERROR_INJECTION_TYPE_RACH = "0" *) 
  (* C_ERROR_INJECTION_TYPE_RDCH = "0" *) 
  (* C_ERROR_INJECTION_TYPE_WACH = "0" *) 
  (* C_ERROR_INJECTION_TYPE_WDCH = "0" *) 
  (* C_ERROR_INJECTION_TYPE_WRCH = "0" *) 
  (* C_FAMILY = "zynq" *) 
  (* C_FULL_FLAGS_RST_VAL = "0" *) 
  (* C_HAS_ALMOST_EMPTY = "0" *) 
  (* C_HAS_ALMOST_FULL = "0" *) 
  (* C_HAS_AXIS_TDATA = "1" *) 
  (* C_HAS_AXIS_TDEST = "0" *) 
  (* C_HAS_AXIS_TID = "0" *) 
  (* C_HAS_AXIS_TKEEP = "0" *) 
  (* C_HAS_AXIS_TLAST = "0" *) 
  (* C_HAS_AXIS_TREADY = "1" *) 
  (* C_HAS_AXIS_TSTRB = "0" *) 
  (* C_HAS_AXIS_TUSER = "1" *) 
  (* C_HAS_AXI_ARUSER = "0" *) 
  (* C_HAS_AXI_AWUSER = "0" *) 
  (* C_HAS_AXI_BUSER = "0" *) 
  (* C_HAS_AXI_ID = "0" *) 
  (* C_HAS_AXI_RD_CHANNEL = "1" *) 
  (* C_HAS_AXI_RUSER = "0" *) 
  (* C_HAS_AXI_WR_CHANNEL = "1" *) 
  (* C_HAS_AXI_WUSER = "0" *) 
  (* C_HAS_BACKUP = "0" *) 
  (* C_HAS_DATA_COUNT = "0" *) 
  (* C_HAS_DATA_COUNTS_AXIS = "0" *) 
  (* C_HAS_DATA_COUNTS_RACH = "0" *) 
  (* C_HAS_DATA_COUNTS_RDCH = "0" *) 
  (* C_HAS_DATA_COUNTS_WACH = "0" *) 
  (* C_HAS_DATA_COUNTS_WDCH = "0" *) 
  (* C_HAS_DATA_COUNTS_WRCH = "0" *) 
  (* C_HAS_INT_CLK = "0" *) 
  (* C_HAS_MASTER_CE = "0" *) 
  (* C_HAS_MEMINIT_FILE = "0" *) 
  (* C_HAS_OVERFLOW = "0" *) 
  (* C_HAS_PROG_FLAGS_AXIS = "0" *) 
  (* C_HAS_PROG_FLAGS_RACH = "0" *) 
  (* C_HAS_PROG_FLAGS_RDCH = "0" *) 
  (* C_HAS_PROG_FLAGS_WACH = "0" *) 
  (* C_HAS_PROG_FLAGS_WDCH = "0" *) 
  (* C_HAS_PROG_FLAGS_WRCH = "0" *) 
  (* C_HAS_RD_DATA_COUNT = "0" *) 
  (* C_HAS_RD_RST = "0" *) 
  (* C_HAS_RST = "0" *) 
  (* C_HAS_SLAVE_CE = "0" *) 
  (* C_HAS_SRST = "1" *) 
  (* C_HAS_UNDERFLOW = "0" *) 
  (* C_HAS_VALID = "0" *) 
  (* C_HAS_WR_ACK = "0" *) 
  (* C_HAS_WR_DATA_COUNT = "0" *) 
  (* C_HAS_WR_RST = "0" *) 
  (* C_IMPLEMENTATION_TYPE = "0" *) 
  (* C_IMPLEMENTATION_TYPE_AXIS = "1" *) 
  (* C_IMPLEMENTATION_TYPE_RACH = "1" *) 
  (* C_IMPLEMENTATION_TYPE_RDCH = "1" *) 
  (* C_IMPLEMENTATION_TYPE_WACH = "1" *) 
  (* C_IMPLEMENTATION_TYPE_WDCH = "1" *) 
  (* C_IMPLEMENTATION_TYPE_WRCH = "1" *) 
  (* C_INIT_WR_PNTR_VAL = "0" *) 
  (* C_INTERFACE_TYPE = "0" *) 
  (* C_MEMORY_TYPE = "1" *) 
  (* C_MIF_FILE_NAME = "BlankString" *) 
  (* C_MSGON_VAL = "1" *) 
  (* C_OPTIMIZATION_MODE = "0" *) 
  (* C_OVERFLOW_LOW = "0" *) 
  (* C_POWER_SAVING_MODE = "0" *) 
  (* C_PRELOAD_LATENCY = "1" *) 
  (* C_PRELOAD_REGS = "0" *) 
  (* C_PRIM_FIFO_TYPE = "4kx4" *) 
  (* C_PRIM_FIFO_TYPE_AXIS = "1kx18" *) 
  (* C_PRIM_FIFO_TYPE_RACH = "512x36" *) 
  (* C_PRIM_FIFO_TYPE_RDCH = "1kx36" *) 
  (* C_PRIM_FIFO_TYPE_WACH = "512x36" *) 
  (* C_PRIM_FIFO_TYPE_WDCH = "1kx36" *) 
  (* C_PRIM_FIFO_TYPE_WRCH = "512x36" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL = "2" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL_AXIS = "1022" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL_RACH = "1022" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL_RDCH = "1022" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL_WACH = "1022" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL_WDCH = "1022" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL_WRCH = "1022" *) 
  (* C_PROG_EMPTY_THRESH_NEGATE_VAL = "3" *) 
  (* C_PROG_EMPTY_TYPE = "0" *) 
  (* C_PROG_EMPTY_TYPE_AXIS = "0" *) 
  (* C_PROG_EMPTY_TYPE_RACH = "0" *) 
  (* C_PROG_EMPTY_TYPE_RDCH = "0" *) 
  (* C_PROG_EMPTY_TYPE_WACH = "0" *) 
  (* C_PROG_EMPTY_TYPE_WDCH = "0" *) 
  (* C_PROG_EMPTY_TYPE_WRCH = "0" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL = "4094" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL_AXIS = "1023" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL_RACH = "1023" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL_RDCH = "1023" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL_WACH = "1023" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL_WDCH = "1023" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL_WRCH = "1023" *) 
  (* C_PROG_FULL_THRESH_NEGATE_VAL = "4093" *) 
  (* C_PROG_FULL_TYPE = "0" *) 
  (* C_PROG_FULL_TYPE_AXIS = "0" *) 
  (* C_PROG_FULL_TYPE_RACH = "0" *) 
  (* C_PROG_FULL_TYPE_RDCH = "0" *) 
  (* C_PROG_FULL_TYPE_WACH = "0" *) 
  (* C_PROG_FULL_TYPE_WDCH = "0" *) 
  (* C_PROG_FULL_TYPE_WRCH = "0" *) 
  (* C_RACH_TYPE = "0" *) 
  (* C_RDCH_TYPE = "0" *) 
  (* C_RD_DATA_COUNT_WIDTH = "12" *) 
  (* C_RD_DEPTH = "4096" *) 
  (* C_RD_FREQ = "1" *) 
  (* C_RD_PNTR_WIDTH = "12" *) 
  (* C_REG_SLICE_MODE_AXIS = "0" *) 
  (* C_REG_SLICE_MODE_RACH = "0" *) 
  (* C_REG_SLICE_MODE_RDCH = "0" *) 
  (* C_REG_SLICE_MODE_WACH = "0" *) 
  (* C_REG_SLICE_MODE_WDCH = "0" *) 
  (* C_REG_SLICE_MODE_WRCH = "0" *) 
  (* C_SELECT_XPM = "0" *) 
  (* C_SYNCHRONIZER_STAGE = "2" *) 
  (* C_UNDERFLOW_LOW = "0" *) 
  (* C_USE_COMMON_OVERFLOW = "0" *) 
  (* C_USE_COMMON_UNDERFLOW = "0" *) 
  (* C_USE_DEFAULT_SETTINGS = "0" *) 
  (* C_USE_DOUT_RST = "1" *) 
  (* C_USE_ECC = "0" *) 
  (* C_USE_ECC_AXIS = "0" *) 
  (* C_USE_ECC_RACH = "0" *) 
  (* C_USE_ECC_RDCH = "0" *) 
  (* C_USE_ECC_WACH = "0" *) 
  (* C_USE_ECC_WDCH = "0" *) 
  (* C_USE_ECC_WRCH = "0" *) 
  (* C_USE_EMBEDDED_REG = "0" *) 
  (* C_USE_FIFO16_FLAGS = "0" *) 
  (* C_USE_FWFT_DATA_COUNT = "0" *) 
  (* C_USE_PIPELINE_REG = "0" *) 
  (* C_VALID_LOW = "0" *) 
  (* C_WACH_TYPE = "0" *) 
  (* C_WDCH_TYPE = "0" *) 
  (* C_WRCH_TYPE = "0" *) 
  (* C_WR_ACK_LOW = "0" *) 
  (* C_WR_DATA_COUNT_WIDTH = "12" *) 
  (* C_WR_DEPTH = "4096" *) 
  (* C_WR_DEPTH_AXIS = "1024" *) 
  (* C_WR_DEPTH_RACH = "16" *) 
  (* C_WR_DEPTH_RDCH = "1024" *) 
  (* C_WR_DEPTH_WACH = "16" *) 
  (* C_WR_DEPTH_WDCH = "1024" *) 
  (* C_WR_DEPTH_WRCH = "16" *) 
  (* C_WR_FREQ = "1" *) 
  (* C_WR_PNTR_WIDTH = "12" *) 
  (* C_WR_PNTR_WIDTH_AXIS = "10" *) 
  (* C_WR_PNTR_WIDTH_RACH = "4" *) 
  (* C_WR_PNTR_WIDTH_RDCH = "10" *) 
  (* C_WR_PNTR_WIDTH_WACH = "4" *) 
  (* C_WR_PNTR_WIDTH_WDCH = "10" *) 
  (* C_WR_PNTR_WIDTH_WRCH = "4" *) 
  (* C_WR_RESPONSE_LATENCY = "1" *) 
  (* is_du_within_envelope = "true" *) 
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fifo_generator_v13_2_8 U0
       (.almost_empty(NLW_U0_almost_empty_UNCONNECTED),
        .almost_full(NLW_U0_almost_full_UNCONNECTED),
        .axi_ar_data_count(NLW_U0_axi_ar_data_count_UNCONNECTED[4:0]),
        .axi_ar_dbiterr(NLW_U0_axi_ar_dbiterr_UNCONNECTED),
        .axi_ar_injectdbiterr(1'b0),
        .axi_ar_injectsbiterr(1'b0),
        .axi_ar_overflow(NLW_U0_axi_ar_overflow_UNCONNECTED),
        .axi_ar_prog_empty(NLW_U0_axi_ar_prog_empty_UNCONNECTED),
        .axi_ar_prog_empty_thresh({1'b0,1'b0,1'b0,1'b0}),
        .axi_ar_prog_full(NLW_U0_axi_ar_prog_full_UNCONNECTED),
        .axi_ar_prog_full_thresh({1'b0,1'b0,1'b0,1'b0}),
        .axi_ar_rd_data_count(NLW_U0_axi_ar_rd_data_count_UNCONNECTED[4:0]),
        .axi_ar_sbiterr(NLW_U0_axi_ar_sbiterr_UNCONNECTED),
        .axi_ar_underflow(NLW_U0_axi_ar_underflow_UNCONNECTED),
        .axi_ar_wr_data_count(NLW_U0_axi_ar_wr_data_count_UNCONNECTED[4:0]),
        .axi_aw_data_count(NLW_U0_axi_aw_data_count_UNCONNECTED[4:0]),
        .axi_aw_dbiterr(NLW_U0_axi_aw_dbiterr_UNCONNECTED),
        .axi_aw_injectdbiterr(1'b0),
        .axi_aw_injectsbiterr(1'b0),
        .axi_aw_overflow(NLW_U0_axi_aw_overflow_UNCONNECTED),
        .axi_aw_prog_empty(NLW_U0_axi_aw_prog_empty_UNCONNECTED),
        .axi_aw_prog_empty_thresh({1'b0,1'b0,1'b0,1'b0}),
        .axi_aw_prog_full(NLW_U0_axi_aw_prog_full_UNCONNECTED),
        .axi_aw_prog_full_thresh({1'b0,1'b0,1'b0,1'b0}),
        .axi_aw_rd_data_count(NLW_U0_axi_aw_rd_data_count_UNCONNECTED[4:0]),
        .axi_aw_sbiterr(NLW_U0_axi_aw_sbiterr_UNCONNECTED),
        .axi_aw_underflow(NLW_U0_axi_aw_underflow_UNCONNECTED),
        .axi_aw_wr_data_count(NLW_U0_axi_aw_wr_data_count_UNCONNECTED[4:0]),
        .axi_b_data_count(NLW_U0_axi_b_data_count_UNCONNECTED[4:0]),
        .axi_b_dbiterr(NLW_U0_axi_b_dbiterr_UNCONNECTED),
        .axi_b_injectdbiterr(1'b0),
        .axi_b_injectsbiterr(1'b0),
        .axi_b_overflow(NLW_U0_axi_b_overflow_UNCONNECTED),
        .axi_b_prog_empty(NLW_U0_axi_b_prog_empty_UNCONNECTED),
        .axi_b_prog_empty_thresh({1'b0,1'b0,1'b0,1'b0}),
        .axi_b_prog_full(NLW_U0_axi_b_prog_full_UNCONNECTED),
        .axi_b_prog_full_thresh({1'b0,1'b0,1'b0,1'b0}),
        .axi_b_rd_data_count(NLW_U0_axi_b_rd_data_count_UNCONNECTED[4:0]),
        .axi_b_sbiterr(NLW_U0_axi_b_sbiterr_UNCONNECTED),
        .axi_b_underflow(NLW_U0_axi_b_underflow_UNCONNECTED),
        .axi_b_wr_data_count(NLW_U0_axi_b_wr_data_count_UNCONNECTED[4:0]),
        .axi_r_data_count(NLW_U0_axi_r_data_count_UNCONNECTED[10:0]),
        .axi_r_dbiterr(NLW_U0_axi_r_dbiterr_UNCONNECTED),
        .axi_r_injectdbiterr(1'b0),
        .axi_r_injectsbiterr(1'b0),
        .axi_r_overflow(NLW_U0_axi_r_overflow_UNCONNECTED),
        .axi_r_prog_empty(NLW_U0_axi_r_prog_empty_UNCONNECTED),
        .axi_r_prog_empty_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .axi_r_prog_full(NLW_U0_axi_r_prog_full_UNCONNECTED),
        .axi_r_prog_full_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .axi_r_rd_data_count(NLW_U0_axi_r_rd_data_count_UNCONNECTED[10:0]),
        .axi_r_sbiterr(NLW_U0_axi_r_sbiterr_UNCONNECTED),
        .axi_r_underflow(NLW_U0_axi_r_underflow_UNCONNECTED),
        .axi_r_wr_data_count(NLW_U0_axi_r_wr_data_count_UNCONNECTED[10:0]),
        .axi_w_data_count(NLW_U0_axi_w_data_count_UNCONNECTED[10:0]),
        .axi_w_dbiterr(NLW_U0_axi_w_dbiterr_UNCONNECTED),
        .axi_w_injectdbiterr(1'b0),
        .axi_w_injectsbiterr(1'b0),
        .axi_w_overflow(NLW_U0_axi_w_overflow_UNCONNECTED),
        .axi_w_prog_empty(NLW_U0_axi_w_prog_empty_UNCONNECTED),
        .axi_w_prog_empty_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .axi_w_prog_full(NLW_U0_axi_w_prog_full_UNCONNECTED),
        .axi_w_prog_full_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .axi_w_rd_data_count(NLW_U0_axi_w_rd_data_count_UNCONNECTED[10:0]),
        .axi_w_sbiterr(NLW_U0_axi_w_sbiterr_UNCONNECTED),
        .axi_w_underflow(NLW_U0_axi_w_underflow_UNCONNECTED),
        .axi_w_wr_data_count(NLW_U0_axi_w_wr_data_count_UNCONNECTED[10:0]),
        .axis_data_count(NLW_U0_axis_data_count_UNCONNECTED[10:0]),
        .axis_dbiterr(NLW_U0_axis_dbiterr_UNCONNECTED),
        .axis_injectdbiterr(1'b0),
        .axis_injectsbiterr(1'b0),
        .axis_overflow(NLW_U0_axis_overflow_UNCONNECTED),
        .axis_prog_empty(NLW_U0_axis_prog_empty_UNCONNECTED),
        .axis_prog_empty_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .axis_prog_full(NLW_U0_axis_prog_full_UNCONNECTED),
        .axis_prog_full_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .axis_rd_data_count(NLW_U0_axis_rd_data_count_UNCONNECTED[10:0]),
        .axis_sbiterr(NLW_U0_axis_sbiterr_UNCONNECTED),
        .axis_underflow(NLW_U0_axis_underflow_UNCONNECTED),
        .axis_wr_data_count(NLW_U0_axis_wr_data_count_UNCONNECTED[10:0]),
        .backup(1'b0),
        .backup_marker(1'b0),
        .clk(clk),
        .data_count(NLW_U0_data_count_UNCONNECTED[11:0]),
        .dbiterr(NLW_U0_dbiterr_UNCONNECTED),
        .din(din),
        .dout(dout),
        .empty(empty),
        .full(full),
        .injectdbiterr(1'b0),
        .injectsbiterr(1'b0),
        .int_clk(1'b0),
        .m_aclk(1'b0),
        .m_aclk_en(1'b0),
        .m_axi_araddr(NLW_U0_m_axi_araddr_UNCONNECTED[31:0]),
        .m_axi_arburst(NLW_U0_m_axi_arburst_UNCONNECTED[1:0]),
        .m_axi_arcache(NLW_U0_m_axi_arcache_UNCONNECTED[3:0]),
        .m_axi_arid(NLW_U0_m_axi_arid_UNCONNECTED[0]),
        .m_axi_arlen(NLW_U0_m_axi_arlen_UNCONNECTED[7:0]),
        .m_axi_arlock(NLW_U0_m_axi_arlock_UNCONNECTED[0]),
        .m_axi_arprot(NLW_U0_m_axi_arprot_UNCONNECTED[2:0]),
        .m_axi_arqos(NLW_U0_m_axi_arqos_UNCONNECTED[3:0]),
        .m_axi_arready(1'b0),
        .m_axi_arregion(NLW_U0_m_axi_arregion_UNCONNECTED[3:0]),
        .m_axi_arsize(NLW_U0_m_axi_arsize_UNCONNECTED[2:0]),
        .m_axi_aruser(NLW_U0_m_axi_aruser_UNCONNECTED[0]),
        .m_axi_arvalid(NLW_U0_m_axi_arvalid_UNCONNECTED),
        .m_axi_awaddr(NLW_U0_m_axi_awaddr_UNCONNECTED[31:0]),
        .m_axi_awburst(NLW_U0_m_axi_awburst_UNCONNECTED[1:0]),
        .m_axi_awcache(NLW_U0_m_axi_awcache_UNCONNECTED[3:0]),
        .m_axi_awid(NLW_U0_m_axi_awid_UNCONNECTED[0]),
        .m_axi_awlen(NLW_U0_m_axi_awlen_UNCONNECTED[7:0]),
        .m_axi_awlock(NLW_U0_m_axi_awlock_UNCONNECTED[0]),
        .m_axi_awprot(NLW_U0_m_axi_awprot_UNCONNECTED[2:0]),
        .m_axi_awqos(NLW_U0_m_axi_awqos_UNCONNECTED[3:0]),
        .m_axi_awready(1'b0),
        .m_axi_awregion(NLW_U0_m_axi_awregion_UNCONNECTED[3:0]),
        .m_axi_awsize(NLW_U0_m_axi_awsize_UNCONNECTED[2:0]),
        .m_axi_awuser(NLW_U0_m_axi_awuser_UNCONNECTED[0]),
        .m_axi_awvalid(NLW_U0_m_axi_awvalid_UNCONNECTED),
        .m_axi_bid(1'b0),
        .m_axi_bready(NLW_U0_m_axi_bready_UNCONNECTED),
        .m_axi_bresp({1'b0,1'b0}),
        .m_axi_buser(1'b0),
        .m_axi_bvalid(1'b0),
        .m_axi_rdata({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .m_axi_rid(1'b0),
        .m_axi_rlast(1'b0),
        .m_axi_rready(NLW_U0_m_axi_rready_UNCONNECTED),
        .m_axi_rresp({1'b0,1'b0}),
        .m_axi_ruser(1'b0),
        .m_axi_rvalid(1'b0),
        .m_axi_wdata(NLW_U0_m_axi_wdata_UNCONNECTED[63:0]),
        .m_axi_wid(NLW_U0_m_axi_wid_UNCONNECTED[0]),
        .m_axi_wlast(NLW_U0_m_axi_wlast_UNCONNECTED),
        .m_axi_wready(1'b0),
        .m_axi_wstrb(NLW_U0_m_axi_wstrb_UNCONNECTED[7:0]),
        .m_axi_wuser(NLW_U0_m_axi_wuser_UNCONNECTED[0]),
        .m_axi_wvalid(NLW_U0_m_axi_wvalid_UNCONNECTED),
        .m_axis_tdata(NLW_U0_m_axis_tdata_UNCONNECTED[7:0]),
        .m_axis_tdest(NLW_U0_m_axis_tdest_UNCONNECTED[0]),
        .m_axis_tid(NLW_U0_m_axis_tid_UNCONNECTED[0]),
        .m_axis_tkeep(NLW_U0_m_axis_tkeep_UNCONNECTED[0]),
        .m_axis_tlast(NLW_U0_m_axis_tlast_UNCONNECTED),
        .m_axis_tready(1'b0),
        .m_axis_tstrb(NLW_U0_m_axis_tstrb_UNCONNECTED[0]),
        .m_axis_tuser(NLW_U0_m_axis_tuser_UNCONNECTED[3:0]),
        .m_axis_tvalid(NLW_U0_m_axis_tvalid_UNCONNECTED),
        .overflow(NLW_U0_overflow_UNCONNECTED),
        .prog_empty(NLW_U0_prog_empty_UNCONNECTED),
        .prog_empty_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .prog_empty_thresh_assert({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .prog_empty_thresh_negate({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .prog_full(NLW_U0_prog_full_UNCONNECTED),
        .prog_full_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .prog_full_thresh_assert({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .prog_full_thresh_negate({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .rd_clk(1'b0),
        .rd_data_count(NLW_U0_rd_data_count_UNCONNECTED[11:0]),
        .rd_en(rd_en),
        .rd_rst(1'b0),
        .rd_rst_busy(NLW_U0_rd_rst_busy_UNCONNECTED),
        .rst(1'b0),
        .s_aclk(1'b0),
        .s_aclk_en(1'b0),
        .s_aresetn(1'b0),
        .s_axi_araddr({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arburst({1'b0,1'b0}),
        .s_axi_arcache({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arid(1'b0),
        .s_axi_arlen({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arlock(1'b0),
        .s_axi_arprot({1'b0,1'b0,1'b0}),
        .s_axi_arqos({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arready(NLW_U0_s_axi_arready_UNCONNECTED),
        .s_axi_arregion({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arsize({1'b0,1'b0,1'b0}),
        .s_axi_aruser(1'b0),
        .s_axi_arvalid(1'b0),
        .s_axi_awaddr({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awburst({1'b0,1'b0}),
        .s_axi_awcache({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awid(1'b0),
        .s_axi_awlen({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awlock(1'b0),
        .s_axi_awprot({1'b0,1'b0,1'b0}),
        .s_axi_awqos({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awready(NLW_U0_s_axi_awready_UNCONNECTED),
        .s_axi_awregion({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awsize({1'b0,1'b0,1'b0}),
        .s_axi_awuser(1'b0),
        .s_axi_awvalid(1'b0),
        .s_axi_bid(NLW_U0_s_axi_bid_UNCONNECTED[0]),
        .s_axi_bready(1'b0),
        .s_axi_bresp(NLW_U0_s_axi_bresp_UNCONNECTED[1:0]),
        .s_axi_buser(NLW_U0_s_axi_buser_UNCONNECTED[0]),
        .s_axi_bvalid(NLW_U0_s_axi_bvalid_UNCONNECTED),
        .s_axi_rdata(NLW_U0_s_axi_rdata_UNCONNECTED[63:0]),
        .s_axi_rid(NLW_U0_s_axi_rid_UNCONNECTED[0]),
        .s_axi_rlast(NLW_U0_s_axi_rlast_UNCONNECTED),
        .s_axi_rready(1'b0),
        .s_axi_rresp(NLW_U0_s_axi_rresp_UNCONNECTED[1:0]),
        .s_axi_ruser(NLW_U0_s_axi_ruser_UNCONNECTED[0]),
        .s_axi_rvalid(NLW_U0_s_axi_rvalid_UNCONNECTED),
        .s_axi_wdata({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_wid(1'b0),
        .s_axi_wlast(1'b0),
        .s_axi_wready(NLW_U0_s_axi_wready_UNCONNECTED),
        .s_axi_wstrb({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_wuser(1'b0),
        .s_axi_wvalid(1'b0),
        .s_axis_tdata({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axis_tdest(1'b0),
        .s_axis_tid(1'b0),
        .s_axis_tkeep(1'b0),
        .s_axis_tlast(1'b0),
        .s_axis_tready(NLW_U0_s_axis_tready_UNCONNECTED),
        .s_axis_tstrb(1'b0),
        .s_axis_tuser({1'b0,1'b0,1'b0,1'b0}),
        .s_axis_tvalid(1'b0),
        .sbiterr(NLW_U0_sbiterr_UNCONNECTED),
        .sleep(1'b0),
        .srst(srst),
        .underflow(NLW_U0_underflow_UNCONNECTED),
        .valid(NLW_U0_valid_UNCONNECTED),
        .wr_ack(NLW_U0_wr_ack_UNCONNECTED),
        .wr_clk(1'b0),
        .wr_data_count(NLW_U0_wr_data_count_UNCONNECTED[11:0]),
        .wr_en(wr_en),
        .wr_rst(1'b0),
        .wr_rst_busy(NLW_U0_wr_rst_busy_UNCONNECTED));
endmodule
`pragma protect begin_protected
`pragma protect version = 1
`pragma protect encrypt_agent = "XILINX"
`pragma protect encrypt_agent_info = "Xilinx Encryption Tool 2023.1"
`pragma protect key_keyowner="Synopsys", key_keyname="SNPS-VCS-RSA-2", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=128)
`pragma protect key_block
aMT3usC6uizzcwnzOCX4OsS16Ob+YxFcsGovFpFklbnaIaD1S0lVdxenTwHPp6ByIEi+ehwr6Rgg
z/3AlTheI5NFTM8ihiMA18/wmUxI7EbaftJACA1LykUKCuj5myy0T+DACuv3sGYIZS38TZTZnnBC
FGAlvTZmRWs+JzneH3o=

`pragma protect key_keyowner="Aldec", key_keyname="ALDEC15_001", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
lR9ZerhYSAb39nzEkeYvhnwEs5t9y/+yTDf8KuoUtR1BGeHZq8pA/YxtjzQLtaOW1R1IQUb0FtSI
e3CYAb7WHYbIjcpw3vKHvW1SqcGn9CMGa556CYKmD2oF12Kow8xRaFvMSBUVxX7HsHxNWnRd+PU1
+C0YayU2KFIY/7Yl6cZ5luAzhw/6SW3PFYUIyyqWy5MCIXweHOwQR2IpQEdlDur5nluN7i7BeB+i
fxwwHh8TU/g7T4mhZFkiTuBKdLAtQOjxWxzqTMxgcuAjlTylY16FgMFOASdvvSbqBZJjbxMdVloU
rYjS8O/8rWktv8GXcaIdBJ2BRj01q7jsChsbwA==

`pragma protect key_keyowner="Mentor Graphics Corporation", key_keyname="MGC-VELOCE-RSA", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=128)
`pragma protect key_block
Qvl63GHz9mq2xOB7elt/vAQ7URLGdD1Lkcz7f3Wtw31dwjjjbP62Ny/Jr6OmBIheWlgejx38qxAT
TrHiiEyjKmGcnPn1Tn2n+cH4RAxCbOFnCI9n6+YsYMTe9JkplGhGGr39SkFgJz0I2IKpPsuqTjCj
rhf49TAryNMQeRpREJA=

`pragma protect key_keyowner="Mentor Graphics Corporation", key_keyname="MGC-VERIF-SIM-RSA-2", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
MA+9Ro+dh339m0iZrkKbqTKN8gQ5xkxN/SPCfhkOn+5jjgCTS5IOKLHil+HsZDjX333ebxnornwG
MOBxyEdFfLM8SA+bs2r41J/j0af2VVMmCM3hOh8JmZxB4X9Jg/glegNCbvwzqxMbOQNEy+zt7j5t
TFVD82RtPFmYVVYZZyll/WvAA+0aVpyjzLCIM1GznFky0RWLv65Wp4MJJnNRRrtG3muMznVO/u2s
tACsJ9jzv9M0IlMYjYH9BixhG6cZX02I4LEXXaPkhdOINlMMhsbArXtc9NphzmS4bY1/1yF1D6YD
EKLyS2Sr3HDl0O/lefN+jvfG8iKuVl55PNNrVQ==

`pragma protect key_keyowner="Real Intent", key_keyname="RI-RSA-KEY-1", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
wpMTg7STjFkUDhOqdNPa0FHXTnHQgKmhvqDv+rRVBvMiQ8O7u8oj7ibITq3o+jugJsMJ60B410gQ
JFTcqCJKYmYJvqi8rPLLOYDmFG6ZLP/Ixr3n62IyIaCeDltBahi3yV009QN0X+iuzuFCL+Y7g9ff
IvAgyBly+Z3Itv2H9EJMZPMl17Sa7IkgjmWqzVXIKNMKn0iDVYsQw6ZgzQDYQ8N8IvTIEggU3/lh
6Nf0hV0ev3qOv/2P+4w0U766Ux3yLuzPJSI7bKm3/ip9NjhOytxOiKKqVXhKG8dzbbuS5u3EE/eq
q6YxkL7gpvNltVqqBnJB6vHSyWrD6+MqsCtR9A==

`pragma protect key_keyowner="Xilinx", key_keyname="xilinxt_2022_10", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
Q7Q4SSp70lxFryaopuic9VVP/Ire0pSsPEIMYdURBAczC7ShkuYeV02U7L3BlAiyBE4vBKcwYSQd
cWiaj8sVP7q4kxoRHKxLV1R5PIO6l4DsLWE2E+1MLyUPME0w5KTular/oX8EPCJ5n/8VCtW7x4Vf
dpeyki1/IAPJkAyi3zVZKHzgKhEwnZaZZtZYuMWoPZMt4V38sAcE42Raf+7yfFWG5HO74JY6iEnW
gJeRk58K+avB/XLF2/j2RQZfjTYizrprT2tUMBK6e7DRWZZtk8AOcsMhUikev44IFGNbNXjP8BXC
0J3y3P7pCFT6l+saU83nRwi/H25fSA34diJtNw==

`pragma protect key_keyowner="Metrics Technologies Inc.", key_keyname="DSim", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
a/8ooC+s+6nfvfa1+oBhsvYWLJjFgp83DI1kNyOi5Am+ugPbGRmgGZudfyo6yw6Yd5gGbLm5aToQ
5G4cGF5HaXD5TU6A0ZZFMTIbzFLE76JMjjIxX8JcaJIZpSmrXqlru8l5gDINUEAmwUY3mRQnjcGJ
0Z+kMRH8iAEF+gEviPiFZSBbJeOPqivIS217kimQJX3BeNbNPQTP+GUidcRywpGMh5avxtA0kDRO
F9SoCSyTm9hr2v9hsK1IUAYQLb7n2/R+z5YNKNzt1oN4qgJH1wZfdI8if2K8+ohyOdnxrrgJOWdj
cOqr7cGqEOYfBMTIQeHVZzb7NGWVN+9B8XSUaQ==

`pragma protect key_keyowner="Atrenta", key_keyname="ATR-SG-RSA-1", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=384)
`pragma protect key_block
FLPvOUNRWNW2GU+FEGmt2XWthOT5bY/31DRbol2cUmEGNF6b2XzpCosNKGx/o2n6sQvGP39KRFCs
nJu0ihe2dUGee9nEZZUcpwPjnEfXVI3yJaRVYy8iL+rm59lXq0jX4sjAPieDvv8shgAnoXLTZGlq
K+2c1JhaHt+nFi27TDrYar/+P8nP1MhocOS7BjzCvSs0foEXj92/qD+71Sm/LqGr8cjlH2qTJJ8B
ynxoH6iT+bksVA2VbtPT9o6h1kJ/zwP4wcsL9l+qSlJhd4GI11JPux26DlNyIi41WmufQcfiT0PB
r6O9+0E9lV9ODwKdjaxfZRK29rjKeq2yr0jWhMV38XKKqHAJli7MIypGRXcCo+u89H87KgYt+ebw
s3foIqCe0JKR57WzI8VD6XdNtOL8eBxK539oemx4vkE0cGYECZKYru6A2hPeZOYDD5eyWSUlQl1R
EciK49WM8HnssyRVcmE6di6bISMbVi0TZG/v98bz+9UZa8DtqMVYH0tz

`pragma protect key_keyowner="Cadence Design Systems.", key_keyname="CDS_RSA_KEY_VER_1", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
fphquQOeFuqByo36Gh2C1zEC1J6u9swSMbMzsKldIvLm+SZ6/hr/N8KJ/G2vBABzX6UtbVuP1ZXx
AxdftP4Aqis1B3Bs6989aQG9eo0SOHA7r6aFLtFb3qoD5Pvqw4aVNU4z4EtTpFpn/jCWD21lKROf
q5X32HRfFq1jwqod+9vIbUNRRzz5y9VHvXfacZlxDazSPmcCF4hxB1KqWqT44KmYVkDedgkgnYgb
ZGidHnTb3W7C8tSqC9ac4kNJCL429QndtddweESJNlpX+65pt9Irok9pkOodwoj0QScswOIFjhBZ
/GrzZLQcFWiD3gXRU4DazzxQnGdRH4qEIRWziw==

`pragma protect key_keyowner="Synplicity", key_keyname="SYNP15_1", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
1lUYYHPCt1BUJOvcBbgMU2GSQiqfxItz4ntieMaenjrtsE9SLwaU6xB0tBl8Atw5yP/RRNww1kX/
9uZbTz5He3r9mPVt+mGxB4N3f9BbCrQRb4USVPgKO/+vWUfMQERGklScy0+fz75WuxH74CjRUoDI
8iyssb2cUNnfDe13jIoI8gM1w4w/Pkxkmb6Mef53QMxacHAWEZeytcH3fuL/adO263D8P90U3XJv
vBXJmbjkRVi9qzjBzfMxuOy2KbZaZgR3BLzaffIfFnMwg/Rb8sGls5pQsZv5jL2wk3+Bj3OXBYdd
pDyjGoalJBzObKzd/t15kNHwY4FXYFcZLQPncw==

`pragma protect key_keyowner="Mentor Graphics Corporation", key_keyname="MGC-PREC-RSA", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
YRmSEzaa2WFVvMH1BwWc1TIUpVbzSEIP0VbI6n0sEgct/X4PiTfMQmK1jBVCaISIzwBxscKQwZOt
mb/nmINGg6I7ih39LSbBMtx6cdCUiyaLkPeRbqfyPpKhvnUIFmdKVvTd1dYzxeOeuDnhSVaBaAcN
3lngSg7lIbmhLIGjC29yQrBTiLArbVZi6IRGronMK51e3UrYa6GspsznhiuRcXjEb4bHKrJ2CM5Z
BUwA+E9949sQgyOagFZbLVle2ESbwBaoxcAPn2gxfRHlT0leqyLgUGDZLsfArzGzw9BTGzyEG2TR
XOrKFNYRfMXMrnGsBM7acIelY4LdAMgsKgDH/A==

`pragma protect data_method = "AES128-CBC"
`pragma protect encoding = (enctype = "BASE64", line_length = 76, bytes = 92336)
`pragma protect data_block
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
`pragma protect end_protected
`ifndef GLBL
`define GLBL
`timescale  1 ps / 1 ps

module glbl ();

    parameter ROC_WIDTH = 100000;
    parameter TOC_WIDTH = 0;
    parameter GRES_WIDTH = 10000;
    parameter GRES_START = 10000;

//--------   STARTUP Globals --------------
    wire GSR;
    wire GTS;
    wire GWE;
    wire PRLD;
    wire GRESTORE;
    tri1 p_up_tmp;
    tri (weak1, strong0) PLL_LOCKG = p_up_tmp;

    wire PROGB_GLBL;
    wire CCLKO_GLBL;
    wire FCSBO_GLBL;
    wire [3:0] DO_GLBL;
    wire [3:0] DI_GLBL;
   
    reg GSR_int;
    reg GTS_int;
    reg PRLD_int;
    reg GRESTORE_int;

//--------   JTAG Globals --------------
    wire JTAG_TDO_GLBL;
    wire JTAG_TCK_GLBL;
    wire JTAG_TDI_GLBL;
    wire JTAG_TMS_GLBL;
    wire JTAG_TRST_GLBL;

    reg JTAG_CAPTURE_GLBL;
    reg JTAG_RESET_GLBL;
    reg JTAG_SHIFT_GLBL;
    reg JTAG_UPDATE_GLBL;
    reg JTAG_RUNTEST_GLBL;

    reg JTAG_SEL1_GLBL = 0;
    reg JTAG_SEL2_GLBL = 0 ;
    reg JTAG_SEL3_GLBL = 0;
    reg JTAG_SEL4_GLBL = 0;

    reg JTAG_USER_TDO1_GLBL = 1'bz;
    reg JTAG_USER_TDO2_GLBL = 1'bz;
    reg JTAG_USER_TDO3_GLBL = 1'bz;
    reg JTAG_USER_TDO4_GLBL = 1'bz;

    assign (strong1, weak0) GSR = GSR_int;
    assign (strong1, weak0) GTS = GTS_int;
    assign (weak1, weak0) PRLD = PRLD_int;
    assign (strong1, weak0) GRESTORE = GRESTORE_int;

    initial begin
	GSR_int = 1'b1;
	PRLD_int = 1'b1;
	#(ROC_WIDTH)
	GSR_int = 1'b0;
	PRLD_int = 1'b0;
    end

    initial begin
	GTS_int = 1'b1;
	#(TOC_WIDTH)
	GTS_int = 1'b0;
    end

    initial begin 
	GRESTORE_int = 1'b0;
	#(GRES_START);
	GRESTORE_int = 1'b1;
	#(GRES_WIDTH);
	GRESTORE_int = 1'b0;
    end

endmodule
`endif
