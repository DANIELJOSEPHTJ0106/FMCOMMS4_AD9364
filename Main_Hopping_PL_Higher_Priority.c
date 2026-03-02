/***************************************************************************/ /**
                                                                               * @file   ad9361/src/main.c
                                                                               * @brief  Implementation of Main Function.
                                                                               * @author DBogdan (dragos.bogdan@analog.com)
                                                                               ********************************************************************************
                                                                               * Copyright 2013(c) Analog Devices, Inc.
                                                                               *
                                                                               * All rights reserved.
                                                                               *
                                                                               * Redistribution and use in source and binary forms, with or without
                                                                               * modification, are permitted provided that the following conditions are met:
                                                                               * - Redistributions of source code must retain the above copyright
                                                                               * notice, this list of conditions and the following disclaimer.
                                                                               * - Redistributions in binary form must reproduce the above copyright
                                                                               * notice, this list of conditions and the following disclaimer in
                                                                               * the documentation and/or other materials provided with the
                                                                               * distribution.
                                                                               * - Neither the name of Analog Devices, Inc. nor the names of its
                                                                               * contributors may be used to endorse or promote products derived
                                                                               * from this software without specific prior written permission.
                                                                               * - The use of this software may or may not infringe the patent rights
                                                                               * of one or more patent holders.  This license does not release you
                                                                               * from the requirement that you obtain separate licenses from these
                                                                               * patent holders to use this software.
                                                                               * - Use of the software either in source or binary form, must be run
                                                                               * on or directly connected to an Analog Devices Inc. component.
                                                                               *
                                                                               * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
                                                                               * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
                                                                               * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
                                                                               * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
                                                                               * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
                                                                               * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
                                                                               * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
                                                                               * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
                                                                               * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
                                                                               * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
                                                                               *******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "ad9361_api.h"
#include "app_config.h"
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "parameters.h"
#include <inttypes.h>

#ifdef XILINX_PLATFORM
#include "axi_sysid.h"
#include "no_os_irq.h"
#include "xilinx_gpio.h"
#include "xilinx_spi.h"
#include <xil_cache.h>
#include <xparameters.h>

#ifdef ALTERA_PLATFORM
#include "altera_gpio.h"
#include "altera_spi.h"

#endif
#endif
#ifdef LINUX_PLATFORM
#include "linux_gpio.h"
#include "linux_spi.h"

#else
#include "xilinx_irq.h"
#endif // LINUX

#include "axi_adc_core.h"
#include "axi_dac_core.h"
#include "axi_dmac.h"
#include "no_os_error.h"
#include "xil_io.h"
#include "xparameters.h"
#include <sleep.h>

#define AXI_GPIO_BASE_ADDRESS XPAR_AXI_GPIO_0_BASEADDR
#define AXI_GPIO_TIME_ADDRESS XPAR_AXI_GPIO_1_BASEADDR
// #include "xil_io.h"  // Direct Memory Read ചെയ്യാൻ ഇത് ആവശ്യമാണ്
//
//// Vivado Address Editor-ൽ നിന്നും നിങ്ങൾക്ക് ലഭിച്ച അഡ്രസ്സ് ഇവിടെ നൽകുക
// #define AXI_GPIO_BASE_ADDRESS 0x4000FFFF
#ifdef IIO_SUPPORT

#include "iio_ad9361.h"
#include "iio_app.h"
#include "iio_axi_adc.h"
#include "iio_axi_dac.h"
#include "no_os_uart.h"

#ifdef XILINX_PLATFORM
#include "xil_cache.h"
#include "xilinx_uart.h"

#endif // XILINX

#if defined LINUX_PLATFORM || defined GENERIC_PLATFORM
static uint8_t in_buff[MAX_SIZE_BASE_ADDR];
static uint8_t out_buff[MAX_SIZE_BASE_ADDR];
#endif

#endif // IIO_SUPPORT

#if defined(DMA_EXAMPLE) || defined(SYSID_BASEADDR)
#include <string.h>
#endif

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/

#if defined(DMA_EXAMPLE) || defined(IIO_SUPPORT)
uint32_t dac_buffer[DAC_BUFFER_SAMPLES] __attribute__((aligned));
#endif
uint16_t adc_buffer[ADC_BUFFER_SAMPLES * ADC_CHANNELS] __attribute__((aligned));

#define AD9361_ADC_DAC_BYTES_PER_SAMPLE 2

#ifdef XILINX_PLATFORM
struct xil_spi_init_param xil_spi_param = {
#ifdef PLATFORM_MB
    .type = SPI_PL,
#else
    .type = SPI_PS,
#endif
    .flags = 0};

struct xil_gpio_init_param xil_gpio_param = {
#ifdef PLATFORM_MB
    .type = GPIO_PL,
#else
    .type = GPIO_PS,
#endif
    .device_id = GPIO_DEVICE_ID};

#define GPIO_OPS &xil_gpio_ops
#define SPI_OPS &xil_spi_ops
#define GPIO_PARAM &xil_gpio_param
#define SPI_PARAM &xil_spi_param
#endif

#ifdef GENERIC_PLATFORM
#define GPIO_OPS &generic_gpio_ops
#define SPI_OPS &generic_spi_ops
#define GPIO_PARAM NULL
#define SPI_PARAM NULL
#endif

#ifdef LINUX_PLATFORM
#define GPIO_OPS &linux_gpio_ops
#define SPI_OPS &linux_spi_ops
#define GPIO_PARAM NULL
#define SPI_PARAM NULL
#endif

struct axi_adc_init rx_adc_init = {.name = "cf-ad9361-lpc",
                                   .base = RX_CORE_BASEADDR,
#ifdef FMCOMMS5
                                   .slave_base = AD9361_RX_1_BASEADDR,
                                   .num_channels = 8,
#else
                                   .num_channels = 4,
#endif
                                   .num_slave_channels = 4};
struct axi_dac_init tx_dac_init = {"cf-ad9361-dds-core-lpc", TX_CORE_BASEADDR,
                                   4, NULL, 3};
struct axi_dmac_init rx_dmac_init = {"rx_dmac", CF_AD9361_RX_DMA_BASEADDR,
#ifdef DMA_IRQ_ENABLE
                                     IRQ_ENABLED
#else
                                     IRQ_DISABLED
#endif
};
struct axi_dmac *rx_dmac;
struct axi_dmac_init tx_dmac_init = {"tx_dmac", CF_AD9361_TX_DMA_BASEADDR,
#ifdef DMA_IRQ_ENABLE
                                     IRQ_ENABLED
#else
                                     IRQ_DISABLED
#endif
};
struct axi_dmac *tx_dmac;

AD9361_InitParam default_init_param = {
    /* Device selection */
    ID_AD9361, // dev_sel
    /* Reference Clock */
    40000000UL, // reference_clk_rate
    /* Base Configuration */
    1, // two_rx_two_tx_mode_enable
    1, // one_rx_one_tx_mode_use_rx_num
    1, // one_rx_one_tx_mode_use_tx_num
    1, // frequency_division_duplex_mode_enable
    0, // frequency_division_duplex_independent_mode_enable
    0, // tdd_use_dual_synth_mode_enable
    0, // tdd_skip_vco_cal_enable
    0, // tx_fastlock_delay_ns
    0, // rx_fastlock_delay_ns
    0, // rx_fastlock_pincontrol_enable
    0, // tx_fastlock_pincontrol_enable
    0, // external_rx_lo_enable
    0, // external_tx_lo_enable
    5, // dc_offset_tracking_update_event_mask
    6, // dc_offset_attenuation_high_range
    5, // dc_offset_attenuation_low_range
    0x28, // dc_offset_count_high_range
    0x32, // dc_offset_count_low_range
    0,    // split_gain_table_mode_enable
    MAX_SYNTH_FREF, // trx_synthesizer_target_fref_overwrite_hz
    0, // qec_tracking_slow_mode_enable
    /* ENSM Control */
    0, // ensm_enable_pin_pulse_mode_enable
    0, // ensm_enable_txnrx_control_enable
    /* LO Control */
    2400000000UL, // rx_synthesizer_frequency_hz
    2400000000UL, // tx_synthesizer_frequency_hz
    1, // tx_lo_powerdown_managed_enable
    /* Rate & BW Control */
    {983040000, 245760000, 122880000, 61440000, 61440000, 61440000}, // rx_path_clock_frequencies[6]
    {983040000, 122880000, 122880000, 61440000, 61440000, 61440000}, // tx_path_clock_frequencies[6]
    18000000,   // rf_rx_bandwidth_hz
    18000000,   // rf_tx_bandwidth_hz
    /* RF Port Control */
    0, // rx_rf_port_input_select
    0, // tx_rf_port_input_select
    /* TX Attenuation Control */
    30500, // tx_attenuation_mdB
    0, // update_tx_gain_in_alert_enable
    /* Reference Clock Control */
    0, // xo_disable_use_ext_refclk_enable
    {8, 5920}, // dcxo_coarse_and_fine_tune[2]
    CLKOUT_DISABLE, // clk_output_mode_select
    /* Gain Control */
    2,    // gc_rx1_mode
    2,    // gc_rx2_mode
    58,   // gc_adc_large_overload_thresh
    4,    // gc_adc_ovr_sample_size
    47,   // gc_adc_small_overload_thresh
    8192, // gc_dec_pow_measurement_duration
    0,   // gc_dig_gain_enable
    800, // gc_lmt_overload_high_thresh
    704, // gc_lmt_overload_low_thresh
    24,  // gc_low_power_thresh
    15,  // gc_max_dig_gain
    0,   // gc_use_rx_fir_out_for_dec_pwr_meas_enable
    /* Gain MGC Control */
    2, // mgc_dec_gain_step
    2, // mgc_inc_gain_step
    0, // mgc_rx1_ctrl_inp_enable
    0, // mgc_rx2_ctrl_inp_enable
    0, // mgc_split_table_ctrl_inp_gain_mode
    /* Gain AGC Control */
    10, // agc_adc_large_overload_exceed_counter
    2, // agc_adc_large_overload_inc_steps
    0, // agc_adc_lmt_small_overload_prevent_gain_inc_enable
    10, // agc_adc_small_overload_exceed_counter
    4, // agc_dig_gain_step_size
    3, // agc_dig_saturation_exceed_counter
    1000, // agc_gain_update_interval_us
    0,    // agc_immed_gain_change_if_large_adc_overload_enable
    0, // agc_immed_gain_change_if_large_lmt_overload_enable
    10, // agc_inner_thresh_high
    1,  // agc_inner_thresh_high_dec_steps
    12, // agc_inner_thresh_low
    1,  // agc_inner_thresh_low_inc_steps
    10, // agc_lmt_overload_large_exceed_counter
    2, // agc_lmt_overload_large_inc_steps
    10, // agc_lmt_overload_small_exceed_counter
    5, // agc_outer_thresh_high
    2, // agc_outer_thresh_high_dec_steps
    18, // agc_outer_thresh_low
    2,  // agc_outer_thresh_low_inc_steps
    1,  // agc_attack_delay_extra_margin_us;
    0, // agc_sync_for_gain_counter_enable
    /* Fast AGC */
    64, // fagc_dec_pow_measuremnt_duration
    260, // fagc_state_wait_time_ns
    /* Fast AGC - Low Power */
    0, // fagc_allow_agc_gain_increase
    5, // fagc_lp_thresh_increment_time
    1, // fagc_lp_thresh_increment_steps
    /* Fast AGC - Lock Level */
    1, // fagc_lock_level_lmt_gain_increase_en
    5, // fagc_lock_level_gain_increase_upper_limit
    /* Fast AGC - Peak Detectors and Final Settling */
    1, // fagc_lpf_final_settling_steps
    1, // fagc_lmt_final_settling_steps
    3, // fagc_final_overrange_count
    /* Fast AGC - Final Power Test */
    0, // fagc_gain_increase_after_gain_lock_en
    /* Fast AGC - Unlocking the Gain */
    0, // fagc_gain_index_type_after_exit_rx_mode
    1, // fagc_use_last_lock_level_for_set_gain_en
    1, // fagc_rst_gla_stronger_sig_thresh_exceeded_en
    5,  // fagc_optimized_gain_offset
    10, // fagc_rst_gla_stronger_sig_thresh_above_ll
    1, // fagc_rst_gla_engergy_lost_sig_thresh_exceeded_en
    1, // fagc_rst_gla_engergy_lost_goto_optim_gain_en
    10, // fagc_rst_gla_engergy_lost_sig_thresh_below_ll
    8, // fagc_energy_lost_stronger_sig_gain_lock_exit_cnt
    1, // fagc_rst_gla_large_adc_overload_en
    1, // fagc_rst_gla_large_lmt_overload_en
    0, // fagc_rst_gla_en_agc_pulled_high_en
    0, // fagc_rst_gla_if_en_agc_pulled_high_mode
    64, // fagc_power_measurement_duration_in_state5
    2, // fagc_large_overload_inc_steps
    /* RSSI Control */
    1,    // rssi_delay
    1000, // rssi_duration
    3,    // rssi_restart_mode
    0, // rssi_unit_is_rx_samples_enable
    1, // rssi_wait
    /* Aux ADC Control */
    256,        // aux_adc_decimation
    40000000UL, // aux_adc_rate
    /* AuxDAC Control */
    1, // aux_dac_manual_mode_enable
    0, // aux_dac1_default_value_mV
    0, // aux_dac1_active_in_rx_enable
    0, // aux_dac1_active_in_tx_enable
    0, // aux_dac1_active_in_alert_enable
    0, // aux_dac1_rx_delay_us
    0, // aux_dac1_tx_delay_us
    0, // aux_dac2_default_value_mV
    0, // aux_dac2_active_in_rx_enable
    0, // aux_dac2_active_in_tx_enable
    0, // aux_dac2_active_in_alert_enable
    0, // aux_dac2_rx_delay_us
    0, // aux_dac2_tx_delay_us
    /* Temperature Sensor Control */
    256,  // temp_sense_decimation
    1000, // temp_sense_measurement_interval_ms
    0xCE, // temp_sense_offset_signed
    1,    // temp_sense_periodic_measurement_enable
    /* Control Out Setup */
    0xFF, // ctrl_outs_enable_mask
    0,    // ctrl_outs_index
    /* External LNA Control */
    0, // elna_settling_delay_ns
    0, // elna_gain_mdB
    0, // elna_bypass_loss_mdB
    0, // elna_rx1_gpo0_control_enable
    0, // elna_rx2_gpo1_control_enable
    0, // elna_gaintable_all_index_enable
    /* Digital Interface Control */
    0, // digital_interface_tune_skip_mode
    0, // digital_interface_tune_fir_disable
    1, // pp_tx_swap_enable
    1, // pp_rx_swap_enable
    0, // tx_channel_swap_enable
    0, // rx_channel_swap_enable
    1, // rx_frame_pulse_mode_enable
    0, // two_t_two_r_timing_enable
    0, // invert_data_bus_enable
    0, // invert_data_clk_enable
    0, // fdd_alt_word_order_enable
    0, // invert_rx_frame_enable
    0, // fdd_rx_rate_2tx_enable
    0, // swap_ports_enable
    0, // single_data_rate_enable
    1, // lvds_mode_enable
    0, // half_duplex_mode_enable
    0, // single_port_mode_enable
    0, // full_port_enable
    0, // full_duplex_swap_bits_enable
    0, // delay_rx_data
    0, // rx_data_clock_delay
    4, // rx_data_delay
    7, // tx_fb_clock_delay
    0, // tx_data_delay
#ifdef ALTERA_PLATFORM
    300, // lvds_bias_mV
#else
    150, // lvds_bias_mV
#endif
    1, // lvds_rx_onchip_termination_enable
    0,    // rx1rx2_phase_inversion_en
    0xFF, // lvds_invert1_control
    0x0F, // lvds_invert2_control
    /* GPO Control */
    0, // gpo_manual_mode_enable
    0, // gpo_manual_mode_enable_mask
    0, // gpo0_inactive_state_high_enable
    0, // gpo1_inactive_state_high_enable
    0, // gpo2_inactive_state_high_enable
    0, // gpo3_inactive_state_high_enable
    0, // gpo0_slave_rx_enable
    0, // gpo0_slave_tx_enable
    0, // gpo1_slave_rx_enable
    0, // gpo1_slave_tx_enable
    0, // gpo2_slave_rx_enable
    0, // gpo2_slave_tx_enable
    0, // gpo3_slave_rx_enable
    0, // gpo3_slave_tx_enable
    0, // gpo0_rx_delay_us
    0, // gpo0_tx_delay_us
    0, // gpo1_rx_delay_us
    0, // gpo1_tx_delay_us
    0, // gpo2_rx_delay_us
    0, // gpo2_tx_delay_us
    0, // gpo3_rx_delay_us
    0, // gpo3_tx_delay_us
    /* Tx Monitor Control */
    37000, // low_high_gain_threshold_mdB
    0,     // low_gain_dB
    24,    // high_gain_dB
    0,     // tx_mon_track_en
    0,     // one_shot_mode_en
    511,   // tx_mon_delay
    8192,  // tx_mon_duration
    2,     // tx1_mon_front_end_gain
    2,     // tx2_mon_front_end_gain
    48,    // tx1_mon_lo_cm
    48,    // tx2_mon_lo_cm
    /* GPIO definitions */
    {.number = -1,
     .platform_ops = GPIO_OPS,
     .extra = GPIO_PARAM}, // gpio_resetb
    /* MCS Sync */
    {.number = -1,
     .platform_ops = GPIO_OPS,
     .extra = GPIO_PARAM}, // gpio_sync

    {.number = -1,
     .platform_ops = GPIO_OPS,
     .extra = GPIO_PARAM}, // gpio_cal_sw1

    {.number = -1,
     .platform_ops = GPIO_OPS,
     .extra = GPIO_PARAM}, // gpio_cal_sw2

    {.device_id = SPI_DEVICE_ID,
     .mode = NO_OS_SPI_MODE_1,
     .chip_select = SPI_CS,
     .platform_ops = SPI_OPS,
     .extra = SPI_PARAM},

    /* External LO clocks */
    NULL, //(*ad9361_rfpll_ext_recalc_rate)()
    NULL, //(*ad9361_rfpll_ext_round_rate)()
    NULL, //(*ad9361_rfpll_ext_set_rate)()
#ifndef AXI_ADC_NOT_PRESENT
    &rx_adc_init, // *rx_adc_init
    &tx_dac_init, // *tx_dac_init
#endif
};

AD9361_RXFIRConfig rx_fir_config = {
    // BPF PASSBAND 3/20 fs to 1/4 fs
    3, // rx
    0, // rx_gain
    1, // rx_dec
    {-4,    -6,    -37,   35,    186,    86,    -284, -315, 107,   219,
     -4,    271,   558,   -307,  -1182,  -356,  658,  157,  207,   1648,
     790,   -2525, -2553, 748,   865,    -476,  3737, 6560, -3583, -14731,
     -5278, 14819, 14819, -5278, -14731, -3583, 6560, 3737, -476,  865,
     748,   -2553, -2525, 790,   1648,   207,   157,  658,  -356,  -1182,
     -307,  558,   271,   -4,    219,    107,   -315, -284, 86,    186,
     35,    -37,   -6,    -4,    0,      0,     0,    0,    0,     0,
     0,     0,     0,     0,     0,      0,     0,    0,    0,     0,
     0,     0,     0,     0,     0,      0,     0,    0,    0,     0,
     0,     0,     0,     0,     0,      0,     0,    0,    0,     0,
     0,     0,     0,     0,     0,      0,     0,    0,    0,     0,
     0,     0,     0,     0,     0,      0,     0,    0,    0,     0,
     0,     0,     0,     0,     0,      0,     0,    0}, // rx_coef[128]
    64,                                                   // rx_coef_size
    {0, 0, 0, 0, 0, 0},                                   // rx_path_clks[6]
    0                                                     // rx_bandwidth
};

AD9361_TXFIRConfig tx_fir_config = {
    // BPF PASSBAND 3/20 fs to 1/4 fs
    3,  // tx
    -6, // tx_gain
    1,  // tx_int
    {-4,    -6,    -37,   35,    186,    86,    -284, -315, 107,   219,
     -4,    271,   558,   -307,  -1182,  -356,  658,  157,  207,   1648,
     790,   -2525, -2553, 748,   865,    -476,  3737, 6560, -3583, -14731,
     -5278, 14819, 14819, -5278, -14731, -3583, 6560, 3737, -476,  865,
     748,   -2553, -2525, 790,   1648,   207,   157,  658,  -356,  -1182,
     -307,  558,   271,   -4,    219,    107,   -315, -284, 86,    186,
     35,    -37,   -6,    -4,    0,      0,     0,    0,    0,     0,
     0,     0,     0,     0,     0,      0,     0,    0,    0,     0,
     0,     0,     0,     0,     0,      0,     0,    0,    0,     0,
     0,     0,     0,     0,     0,      0,     0,    0,    0,     0,
     0,     0,     0,     0,     0,      0,     0,    0,    0,     0,
     0,     0,     0,     0,     0,      0,     0,    0,    0,     0,
     0,     0,     0,     0,     0,      0,     0,    0}, // tx_coef[128]
    64,                                                   // tx_coef_size
    {0, 0, 0, 0, 0, 0},                                   // tx_path_clks[6]
    0                                                     // tx_bandwidth
};

struct ad9361_rf_phy *ad9361_phy;
#ifdef FMCOMMS5
struct ad9361_rf_phy *ad9361_phy_b;
#endif

// ====================================================================
// GLOBAL VARIABLES AND FUNCTIONS FOR WEIGHTED HOPPING
// ഈ ഭാഗം main() ന് മുകളിൽ നൽകിയത് ശ്രദ്ധിക്കുക
// ====================================================================
#define NUM_PROFILES 120
uint64_t my_120_freqs[NUM_PROFILES];
uint8_t fastlock_saved_data[NUM_PROFILES][16];
int freq_weights[NUM_PROFILES]; // Weight സേവ് ചെയ്യാനുള്ള അറേ

int current_freq_index = 0;
int current_weight_count = 0;

// അടുത്തതായി ലോഡ് ചെയ്യേണ്ട പ്രൊഫൈൽ ഏതാണെന്ന് തീരുമാനിക്കുന്ന ഫംഗ്ഷൻ
int get_weighted_profile() {
    int profile_to_load = current_freq_index;

    current_weight_count++;

    // നൽകിയിട്ടുള്ള Weight-ന് തുല്യമായ തവണ ഈ ഫ്രീക്വൻസി ആവർത്തിച്ചു കഴിഞ്ഞാൽ മാത്രം അടുത്തതിലേക്ക് പോകുക
    if (current_weight_count >= freq_weights[current_freq_index]) {
        current_weight_count = 0; // റീസെറ്റ് ചെയ്യുന്നു
        current_freq_index++;     // അടുത്ത ഫ്രീക്വൻസിയിലേക്ക് മാറുന്നു

        if (current_freq_index >= NUM_PROFILES) {
            current_freq_index = 0; // 120 കഴിഞ്ഞാൽ വീണ്ടും 0 ൽ നിന്നും തുടങ്ങാൻ
        }
    }

    return profile_to_load;
}
///////////////////////////////////////////////////////
int main(void) {
  int32_t status;
#ifdef XILINX_PLATFORM
  Xil_ICacheEnable();
  Xil_DCacheEnable();
  default_init_param.spi_param.extra = &xil_spi_param;
  default_init_param.spi_param.platform_ops = &xil_spi_ops;
#endif

#ifdef ALTERA_PLATFORM
  default_init_param.spi_param.platform_ops = &altera_spi_ops;

  if (altera_bridge_init()) {
    printf("Altera Bridge Init Error!\n");
    return -1;
  }
#endif

  // NOTE: The user has to choose the GPIO numbers according to desired
  // carrier board.
  default_init_param.gpio_resetb.number = GPIO_RESET_PIN;

#ifdef FMCOMMS5
  default_init_param.gpio_sync.number = GPIO_SYNC_PIN;
  default_init_param.gpio_cal_sw1.number = GPIO_CAL_SW1_PIN;
  default_init_param.gpio_cal_sw2.number = GPIO_CAL_SW2_PIN;
  default_init_param.rx1rx2_phase_inversion_en = 1;
#else
  default_init_param.gpio_sync.number = -1;
  default_init_param.gpio_cal_sw1.number = -1;
  default_init_param.gpio_cal_sw2.number = -1;
#endif

  if (AD9364_DEVICE) {
    default_init_param.dev_sel = ID_AD9364;
    tx_dac_init.num_channels = 2;
    tx_dac_init.rate = 1;
    rx_adc_init.num_channels = 2;
    rx_adc_init.num_slave_channels = 0;
  } else {
    if (!default_init_param.two_rx_two_tx_mode_enable) {
      tx_dac_init.num_channels = 2;
      tx_dac_init.rate = 1;
      rx_adc_init.num_channels = 2;
      rx_adc_init.num_slave_channels = 0;
    }
  }
  if (AD9363A_DEVICE)
    default_init_param.dev_sel = ID_AD9363A;

#if defined FMCOMMS5 || defined ADI_RF_SOM || defined ADI_RF_SOM_CMOS
  default_init_param.xo_disable_use_ext_refclk_enable = 1;
#endif

#ifdef ADI_RF_SOM_CMOS
  if (AD9361_DEVICE)
    default_init_param.swap_ports_enable = 1;
  default_init_param.lvds_mode_enable = 0;
  default_init_param.lvds_rx_onchip_termination_enable = 0;
  default_init_param.full_port_enable = 1;
  default_init_param.digital_interface_tune_fir_disable = 1;
#endif

  ad9361_init(&ad9361_phy, &default_init_param);

  ad9361_set_tx_fir_config(ad9361_phy, tx_fir_config);
  ad9361_set_rx_fir_config(ad9361_phy, rx_fir_config);

  //////////////////////////////////////////////////////////////////////////////////////////////////
  //	 /* ================================================================
  //	* TX FASTLOCK PROFILE SETUP
  //	* Must be done AFTER ad9361_init() and FIR config
  //	* Store PLL calibration data for 4 TX hop frequencies
  //	* ================================================================ */
  //
  //	// Profile 0: 2400 MHz
  //	ad9361_set_tx_lo_freq(ad9361_phy, 100000000ULL);
  //	no_os_mdelay(2);  // wait for PLL to lock fully
  //	ad9361_tx_fastlock_store(ad9361_phy, 0);
  //
  //	// Profile 1: 2450 MHz
  //	ad9361_set_tx_lo_freq(ad9361_phy, 150000000ULL);
  //	no_os_mdelay(2);
  //	ad9361_tx_fastlock_store(ad9361_phy, 1);
  //
  //	// Profile 2: 2500 MHz
  //	ad9361_set_tx_lo_freq(ad9361_phy, 200000000ULL);
  //	no_os_mdelay(2);
  //	ad9361_tx_fastlock_store(ad9361_phy, 2);
  //
  //	// Profile 3: 2550 MHz
  //	ad9361_set_tx_lo_freq(ad9361_phy, 250000000ULL);
  //	no_os_mdelay(2);
  //	ad9361_tx_fastlock_store(ad9361_phy, 3);
  //
  //	// Profile 4: 2600 MHz
  //	ad9361_set_tx_lo_freq(ad9361_phy, 300000000ULL);
  //	no_os_mdelay(2);
  //	ad9361_tx_fastlock_store(ad9361_phy, 4);
  //
  //	// Profile 5: 2650 MHz
  //	ad9361_set_tx_lo_freq(ad9361_phy, 350000000ULL);
  //	no_os_mdelay(2);
  //	ad9361_tx_fastlock_store(ad9361_phy, 5);
  //
  //	// Profile 6: 2700 MHz
  //	ad9361_set_tx_lo_freq(ad9361_phy, 450000000ULL);
  //	no_os_mdelay(2);
  //	ad9361_tx_fastlock_store(ad9361_phy, 6);
  //
  //	// Profile 6: 2750 MHz
  //	ad9361_set_tx_lo_freq(ad9361_phy, 500000000ULL);
  //	no_os_mdelay(2);
  //	ad9361_tx_fastlock_store(ad9361_phy, 7);
  //
  //	// Return TX LO to starting frequency before DMA begins
  //	ad9361_set_tx_lo_freq(ad9361_phy, 100000000ULL);
  //	no_os_mdelay(2);
  //
  //	ad9361_spi_write(ad9361_phy->spi, 0x29A, 0x03);
  //	printf("TX FastLock: 8 profiles stored (2400/2450/2500/2550
  // MHz.....)\n");
  /* ================================================================ */
  ////////////////////////////////////////////////////////////////////////////////////////////////
  //	uint64_t my_20_freqs[20] = {
  //		    2400000000ULL, 2450000000ULL, 2500000000ULL, 2550000000ULL,
  //			2600000000ULL, 2650000000ULL, 2700000000ULL,
  // 2750000000ULL, 			2800000000ULL, 2850000000ULL,
  // 2900000000ULL, 2950000000ULL, 		    3000000000ULL, 3050000000ULL, 3100000000ULL,
  //3150000000ULL, 		    3200000000ULL, 3250000000ULL, 3300000000ULL, 3350000000ULL
  //		};
  //		uint8_t fastlock_saved_data[20][16];
  //		struct no_os_gpio_desc *group_flag_gpio;
  //		struct no_os_gpio_init_param gpio_init_param = {
  //		    .number = 0,
  //		    .platform_ops = GPIO_OPS,
  //		    .extra = GPIO_PARAM
  //		};
  //
  //		no_os_gpio_get(&group_flag_gpio, &gpio_init_param);
  //		no_os_gpio_direction_input(group_flag_gpio);
  //		printf("Calibrating 20 profiles...\n");
  //		for (int i = 0; i < 20; i++) {
  //		    ad9361_set_tx_lo_freq(ad9361_phy, my_20_freqs[i]);
  //		    no_os_mdelay(2);
  //		    ad9361_tx_fastlock_store(ad9361_phy, 0);
  //		    ad9361_tx_fastlock_save(ad9361_phy, 0,
  // fastlock_saved_data[i]);
  //		}
  //		printf("Loading initial 8 profiles...\n");
  //		for (int slot = 0; slot < 8; slot++) {
  //		    ad9361_tx_fastlock_load(ad9361_phy, slot,
  // fastlock_saved_data[slot]);
  //		}
  ///		ad9361_spi_write(ad9361_phy->spi, 0x29A, 0x03);
  //	    ad9361_set_tx_lo_freq(ad9361_phy, my_20_freqs[0]);
  //	    no_os_mdelay(2);
  ////////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////////////////////
  //	    #define NUM_PROFILES 120
  //	    uint64_t my_120_freqs[NUM_PROFILES] = {
  //	        100000000ULL, 150000000ULL, 200000000ULL, 250000000ULL,
  //	        300000000ULL, 350000000ULL, 400000000ULL, 450000000ULL,
  //	        500000000ULL, 550000000ULL, 600000000ULL, 650000000ULL,
  //	        700000000ULL, 750000000ULL, 800000000ULL, 850000000ULL,
  //	        900000000ULL, 950000000ULL, 1000000000ULL, 1050000000ULL,
  //	        1100000000ULL, 1150000000ULL, 1200000000ULL, 1250000000ULL,
  //	        1300000000ULL, 1350000000ULL, 1400000000ULL, 1450000000ULL,
  //	        1500000000ULL, 1550000000ULL, 1600000000ULL, 1650000000ULL,
  //	        1700000000ULL, 1750000000ULL, 1800000000ULL, 1850000000ULL,
  //	        1900000000ULL, 1950000000ULL, 2000000000ULL, 2050000000ULL,
  //	        2100000000ULL, 2150000000ULL, 2200000000ULL, 2250000000ULL,
  //	        2300000000ULL, 2350000000ULL, 2400000000ULL, 2450000000ULL,
  //	        2500000000ULL, 2550000000ULL, 2600000000ULL, 2650000000ULL,
  //	        2700000000ULL, 2750000000ULL, 2800000000ULL, 2850000000ULL,
  //	        2900000000ULL, 2950000000ULL, 3000000000ULL, 3050000000ULL,
  //	        3100000000ULL, 3150000000ULL, 3200000000ULL, 3250000000ULL,
  //	        3300000000ULL, 3350000000ULL, 3400000000ULL, 3450000000ULL,
  //	        3500000000ULL, 3550000000ULL, 3600000000ULL, 3650000000ULL,
  //	        3700000000ULL, 3750000000ULL, 3800000000ULL, 3850000000ULL,
  //	        3900000000ULL, 3950000000ULL, 4000000000ULL, 4050000000ULL,
  //	        4100000000ULL, 4150000000ULL, 4200000000ULL, 4250000000ULL,
  //	        4300000000ULL, 4350000000ULL, 4400000000ULL, 4450000000ULL,
  //	        4500000000ULL, 4550000000ULL, 4600000000ULL, 4650000000ULL,
  //	        4700000000ULL, 4750000000ULL, 4800000000ULL, 4850000000ULL,
  //	        4900000000ULL, 4950000000ULL, 5000000000ULL, 5050000000ULL,
  //	        5100000000ULL, 5150000000ULL, 5200000000ULL, 5250000000ULL,
  //	        5300000000ULL, 5350000000ULL, 5400000000ULL, 5450000000ULL,
  //	        5500000000ULL, 5550000000ULL, 5600000000ULL, 5650000000ULL,
  //	        5700000000ULL, 5750000000ULL, 5800000000ULL, 5850000000ULL,
  //	        5900000000ULL, 5950000000ULL, 6000000000ULL, 6050000000ULL
  //	    };
  //
  //	    uint8_t fastlock_saved_data[NUM_PROFILES][16];
  //	    struct no_os_gpio_desc *group_flag_gpio;
  //	    struct no_os_gpio_init_param gpio_init_param = {
  //	        .number = 0,
  //	        .platform_ops = GPIO_OPS,
  //	        .extra = GPIO_PARAM
  //	    };
  //	    no_os_gpio_get(&group_flag_gpio, &gpio_init_param);
  //	    no_os_gpio_direction_input(group_flag_gpio);
  //	    printf("Calibrating %d profiles manually...\n", NUM_PROFILES);
  //	    for (int i = 0; i < NUM_PROFILES; i++) {
  //	        ad9361_set_tx_lo_freq(ad9361_phy, my_120_freqs[i]);
  //	        no_os_mdelay(2);
  //	        ad9361_tx_fastlock_store(ad9361_phy, 0);
  //	        ad9361_tx_fastlock_save(ad9361_phy, 0, fastlock_saved_data[i]);
  //	    }
  //	    printf("Loading initial 8 profiles...\n");
  //	    for (int slot = 0; slot < 8; slot++) {
  //	        ad9361_tx_fastlock_load(ad9361_phy, slot,
  // fastlock_saved_data[slot]);
  //	    }
  //	    ad9361_spi_write(ad9361_phy->spi, 0x29A, 0x03);
  //	    ad9361_set_tx_lo_freq(ad9361_phy, my_120_freqs[0]);
  //	    no_os_mdelay(2);
  //////////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////////////////////////
//#define NUM_PROFILES 120
//  uint64_t my_120_freqs[NUM_PROFILES];
//  uint8_t fastlock_saved_data[NUM_PROFILES][16];
//  uint64_t start_freq = 100000000ULL; // 100 MHz
//  uint64_t end_freq = 6050000000ULL;  // 6 GHz
//  uint64_t step_size = 50000000ULL;
//
//  for (int i = 0; i < NUM_PROFILES; i++) {
//    my_120_freqs[i] = start_freq + (i * step_size);
//  }
//  struct no_os_gpio_desc *group_flag_gpio;
//  struct no_os_gpio_init_param gpio_init_param = {
//      .number = 0, .platform_ops = GPIO_OPS, .extra = GPIO_PARAM};
//  no_os_gpio_get(&group_flag_gpio, &gpio_init_param);
//  no_os_gpio_direction_input(group_flag_gpio);
//  printf("Calibrating 120 profiles...\n");
//  for (int i = 0; i < NUM_PROFILES; i++) {
//    ad9361_set_tx_lo_freq(ad9361_phy, my_120_freqs[i]);
//    no_os_mdelay(20);
//    ad9361_tx_fastlock_store(ad9361_phy, 0);
//    ad9361_tx_fastlock_save(ad9361_phy, 0, fastlock_saved_data[i]);
//  }
//  printf("Loading initial 8 profiles...\n");
//  for (int slot = 0; slot < 8; slot++) {
//    ad9361_tx_fastlock_load(ad9361_phy, slot, fastlock_saved_data[slot]);
//  }
//  ad9361_spi_write(ad9361_phy->spi, 0x29A, 0x03);
//  ad9361_set_tx_lo_freq(ad9361_phy, my_120_freqs[0]);
//  no_os_mdelay(2);
//////////////////////////////////////////////////////////////////////////////////////////////////
  // ====================================================================
    // FREQUENCY & WEIGHT SETUP
    // ====================================================================
    uint64_t start_freq = 100000000ULL; // 100 MHz
    uint64_t end_freq = 6050000000ULL;  // 6 GHz
    uint64_t step_size = 50000000ULL;
    uint64_t priority_freqs[10] = {
        800000000ULL,   // 800 MHz
        900000000ULL,   // 900 MHz
        1200000000ULL,  // 1.2 GHz
        2400000000ULL,  // 2.4 GHz
        2500000000ULL,  // 2.5 GHz
        3000000000ULL,  // 3.0 GHz
        4500000000ULL,  // 4.5 GHz
        5000000000ULL,  // 5.0 GHz
        5500000000ULL,  // 5.5 GHz
        5800000000ULL   // 5.8 GHz
    };

    for (int i = 0; i < NUM_PROFILES; i++) {
        my_120_freqs[i] = start_freq + (i * step_size);

        freq_weights[i] = 1; // ആദ്യം എല്ലാത്തിനും സാധാരണ സമയം (Weight 1) നൽകുന്നു

        // ഈ ഫ്രീക്വൻസി നമ്മുടെ Priority ലിസ്റ്റിൽ ഉണ്ടോ എന്ന് പരിശോധിക്കുന്നു
        for (int j = 0; j < 10; j++) {
            if (my_120_freqs[i] == priority_freqs[j]) {
                freq_weights[i] = 5; // ലിസ്റ്റിൽ ഉണ്ടെങ്കിൽ Weight 5 ആക്കി മാറ്റുന്നു!
                break;
            }
        }
    }

    struct no_os_gpio_desc *group_flag_gpio;
    struct no_os_gpio_init_param gpio_init_param = {
        .number = 0, .platform_ops = GPIO_OPS, .extra = GPIO_PARAM};
    no_os_gpio_get(&group_flag_gpio, &gpio_init_param);
    no_os_gpio_direction_input(group_flag_gpio);

    printf("Calibrating 120 profiles...\n");
    for (int i = 0; i < NUM_PROFILES; i++) {
      ad9361_set_tx_lo_freq(ad9361_phy, my_120_freqs[i]);
      no_os_mdelay(20);
      ad9361_tx_fastlock_store(ad9361_phy, 0);
      ad9361_tx_fastlock_save(ad9361_phy, 0, fastlock_saved_data[i]);
    }

    printf("Loading initial 8 profiles with Dwell Time Weighting...\n");
    for (int slot = 0; slot < 8; slot++) {
      int profile_idx = get_weighted_profile(); // പുതിയ ഫംഗ്ഷൻ വിളിക്കുന്നു
      ad9361_tx_fastlock_load(ad9361_phy, slot, fastlock_saved_data[profile_idx]);
    }

    ad9361_spi_write(ad9361_phy->spi, 0x29A, 0x03);
    ad9361_set_tx_lo_freq(ad9361_phy, my_120_freqs[0]);
    no_os_mdelay(2);
    // ====================================================================

  //////////////////////////////////////////////////////////////////////////////////////////////
#ifdef FMCOMMS5
#ifdef LINUX_PLATFORM
  gpio_init(default_init_param.gpio_sync);
#endif
  default_init_param.spi_param.chip_select = SPI_CS_2;
  default_init_param.gpio_resetb.number = GPIO_RESET_PIN_2;
#ifdef LINUX_PLATFORM
  gpio_init(default_init_param.gpio_resetb);
#endif
  default_init_param.gpio_sync.number = -1;
  default_init_param.gpio_cal_sw1.number = -1;
  default_init_param.gpio_cal_sw2.number = -1;
  default_init_param.rx_synthesizer_frequency_hz = 2300000000UL;
  default_init_param.tx_synthesizer_frequency_hz = 2300000000UL;

  rx_adc_init.base = AD9361_RX_1_BASEADDR;
  rx_adc_init.num_slave_channels = 0;
  tx_dac_init.base = AD9361_TX_1_BASEADDR;

  ad9361_init(&ad9361_phy_b, &default_init_param);

  ad9361_set_tx_fir_config(ad9361_phy_b, tx_fir_config);
  ad9361_set_rx_fir_config(ad9361_phy_b, rx_fir_config);
#endif
  status = axi_dmac_init(&tx_dmac, &tx_dmac_init);
  if (status < 0) {
    printf("axi_dmac_init tx init error: %" PRIi32 "\n", status);
    return status;
  }
  status = axi_dmac_init(&rx_dmac, &rx_dmac_init);
  if (status < 0) {
    printf("axi_dmac_init rx init error: %" PRIi32 "\n", status);
    return status;
  }
#ifndef AXI_ADC_NOT_PRESENT
#if defined XILINX_PLATFORM || defined LINUX_PLATFORM || defined ALTERA_PLATFORM
#ifdef DMA_EXAMPLE
#ifdef FMCOMMS5
  axi_dac_init(&ad9361_phy_b->tx_dac, &tx_dac_init);
  axi_dac_set_datasel(ad9361_phy_b->tx_dac, -1, AXI_DAC_DATA_SEL_DMA);
  rx_adc_init.base = AD9361_RX_0_BASEADDR;
  rx_adc_init.num_slave_channels = 4;
  tx_dac_init.base = AD9361_TX_0_BASEADDR;
#endif
  axi_dac_init(&ad9361_phy->tx_dac, &tx_dac_init);
  extern const uint32_t sine_lut_iq[1024];
  axi_dac_set_datasel(ad9361_phy->tx_dac, -1, AXI_DAC_DATA_SEL_DMA);
  axi_dac_load_custom_data(ad9361_phy->tx_dac, sine_lut_iq,
                           NO_OS_ARRAY_SIZE(sine_lut_iq),
                           (uintptr_t)dac_buffer);
#ifdef XILINX_PLATFORM
  Xil_DCacheFlush();
#endif
#else
#ifdef FMCOMMS5
  axi_dac_init(&ad9361_phy_b->tx_dac, &tx_dac_init);
  axi_dac_set_datasel(ad9361_phy_b->tx_dac, -1, AXI_DAC_DATA_SEL_DDS);
  rx_adc_init.base = AD9361_RX_0_BASEADDR;
  rx_adc_init.num_slave_channels = 4;
  tx_dac_init.base = AD9361_TX_0_BASEADDR;
#endif
  axi_dac_init(&ad9361_phy->tx_dac, &tx_dac_init);
  axi_dac_set_datasel(ad9361_phy->tx_dac, -1, AXI_DAC_DATA_SEL_DDS);
#endif
#endif
#endif

#ifdef FMCOMMS5
  ad9361_do_mcs(ad9361_phy, ad9361_phy_b);
#endif

#ifndef AXI_ADC_NOT_PRESENT
#if (defined XILINX_PLATFORM || defined ALTERA_PLATFORM)
  uint32_t samples = 16384;
#if (defined DMA_IRQ_ENABLE)
  /**
   * Xilinx platform dependent initialization for IRQ.
   */
  struct xil_irq_init_param xil_irq_init_par = {
      .type = IRQ_PS,
  };

  /**
   * IRQ initial configuration.
   */
  struct no_os_irq_init_param irq_init_param = {
      .irq_ctrl_id = INTC_DEVICE_ID,
      .platform_ops = &xil_irq_ops,
      .extra = &xil_irq_init_par,
  };

  /**
   * IRQ instance.
   */
  struct no_os_irq_ctrl_desc *irq_desc;

  status = no_os_irq_ctrl_init(&irq_desc, &irq_init_param);
  if (status < 0)
    return status;

  status = no_os_irq_global_enable(irq_desc);
  if (status < 0)
    return status;

  struct no_os_callback_desc rx_dmac_callback = {
      .ctx = rx_dmac,
      .callback = axi_dmac_dev_to_mem_isr,
  };

  status = no_os_irq_register_callback(irq_desc, AD9361_ADC_DMA_IRQ_INTR,
                                       &rx_dmac_callback);
  if (status < 0)
    return status;

  status = no_os_irq_trigger_level_set(irq_desc, AD9361_ADC_DMA_IRQ_INTR,
                                       NO_OS_IRQ_LEVEL_HIGH);
  if (status < 0)
    return status;

  status = no_os_irq_enable(irq_desc, AD9361_ADC_DMA_IRQ_INTR);
  if (status < 0)
    return status;

  samples = 2048;
#endif
  // NOTE: To prevent unwanted data loss, it's recommended to invalidate
  // cache after each axi_dmac_transfer_start() call, keeping in mind that the
  // size of the capture and the start address must be aligned to the size
  // of the cache line.

#ifdef DMA_EXAMPLE
#ifdef DMA_IRQ_ENABLE
  struct no_os_callback_desc tx_dmac_callback = {
      .ctx = tx_dmac,
      .callback = axi_dmac_mem_to_dev_isr,
  };

  status = no_os_irq_register_callback(irq_desc, AD9361_DAC_DMA_IRQ_INTR,
                                       &tx_dmac_callback);
  if (status < 0)
    return status;

  status = no_os_irq_enable(irq_desc, AD9361_DAC_DMA_IRQ_INTR);
  if (status < 0)
    return status;
#endif

  struct axi_dma_transfer transfer = {// Number of bytes to write/read
                                      .size = sizeof(sine_lut_iq),
                                      // Transfer done flag
                                      .transfer_done = 0,
                                      // Signal transfer mode
                                      .cyclic = CYCLIC,
                                      // Address of data source
                                      .src_addr = (uintptr_t)dac_buffer,
                                      // Address of data destination
                                      .dest_addr = 0};

  /* Transfer the data. */
  axi_dmac_transfer_start(tx_dmac, &transfer);

  /* Flush cache data. */
  Xil_DCacheInvalidateRange((uintptr_t)dac_buffer, sizeof(sine_lut_iq));

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  //	/* ================================================================
  //	* TX FASTLOCK HOPPING LOOP
  //	* TX DMA cyclic transfer is already running above (autonomous).
  //	* This loop hops the TX LO between pre-calibrated frequencies.
  //	* Each hop takes ~4us. Dwell time = no_os_mdelay(1) = ~1ms per freq.
  //	* At 4 profiles x 1ms dwell -> cycles through ~250 times/sec.
  //	* Reduce dwell to no_os_udelay(250) for ~1000 hops/sec.
  //	* ================================================================ */
  //	{
  //	uint32_t hop_profile = 0;
  //	const uint64_t hop_freqs[6] = {
  //	2400000000ULL,
  //	2450000000ULL,
  //	2500000000ULL,
  //	2550000000ULL,
  //	2600000000ULL,
  //	2650000000ULL,
  //	2700000000ULL
  //	};
  //	uint32_t hop_count = 0;
  //
  //	printf("TX FastLock hopping started...\n");
  //
  //	while (1) {
  //	/* Instantly recall pre-calibrated TX PLL profile */
  //	ad9361_tx_fastlock_recall(ad9361_phy, hop_profile);
  //
  //	/* Dwell time at this frequency before next hop */
  //	/* Use no_os_udelay(250) for ~1000 hops/sec      */
  //	no_os_mdelay(1);
  //
  //	/* Advance to next profile, wrap 0-1-2-3-4-...-7-0 */
  //	hop_profile = (hop_profile + 1) % 7;
  //
  //	/* Print every 1000 hops so you can monitor */
  //	hop_count++;
  //	if (hop_count % 15 == 0) {
  //	printf("Hopped %lu times. Current profile: %lu (%.0f MHz)\n",
  //	      (unsigned long)hop_count,
  //	      (unsigned long)hop_profile,
  //	      (double)hop_freqs[hop_profile] / 1e6);
  //	}
  //	}
  //	}
  //	/* ================================================================ */
  //
  ////////////////////////////////////////////////////////////////////////////////////////////////////
//  // ====================================================================
//  // 2. PING-PONG FASTLOCK HOPPING LOOP
//  // ====================================================================
//  int next_profile_index = 8;
//  uint32_t flag_value = 0;
//  int loop_count = 1;
//  Xil_Out32(AXI_GPIO_TIME_ADDRESS, 99999999);
//  printf("Starting Ping-Pong Fastlock Hopping...\n");
//
//  while (1) {
//    do {
//      flag_value = Xil_In32(AXI_GPIO_BASE_ADDRESS) & 0x01;
//    } while (flag_value == 0);
//
//    // printf("Loop %d: FPGA is in Group B. Loading Frequencies to Group
//    // A...\n", loop_count);
//
//    for (int slot = 0; slot < 4; slot++) {
//      ad9361_tx_fastlock_load(ad9361_phy, slot,
//                              fastlock_saved_data[next_profile_index]);
//      next_profile_index++;
//      if (next_profile_index >= NUM_PROFILES)
//        next_profile_index = 0;
//    }
//    do {
//      flag_value = Xil_In32(AXI_GPIO_BASE_ADDRESS) & 0x01;
//    } while (flag_value != 0);
//
//    // printf("Loop %d: FPGA is in Group A. Loading Frequencies to Group
//    // B...\n", loop_count);
//
//    for (int slot = 4; slot < 8; slot++) {
//      ad9361_tx_fastlock_load(ad9361_phy, slot,
//                              fastlock_saved_data[next_profile_index]);
//      next_profile_index++;
//      if (next_profile_index >= NUM_PROFILES)
//        next_profile_index = 0;
//    }
//
//    loop_count++;
//  }
////////////////////////////////////////////////////////////////////////////////////////////////////
  // ====================================================================
   // PING-PONG FASTLOCK HOPPING LOOP (With Dwell Time Priority)
   // ====================================================================
   uint32_t flag_value = 0;
   int loop_count = 1;

   // വേഗത സെറ്റ് ചെയ്യുന്നു.
   // 3999 നൽകിയാൽ ഒരു സാധാരണ സ്ലോട്ടിന് 40us സമയം എടുക്കും.
   // അതായത് Priority ഉള്ളതിന് 5 x 40us = 200us സമയം ലഭിക്കും.
   Xil_Out32(AXI_GPIO_TIME_ADDRESS, 99999999);
   printf("Starting Weighted Ping-Pong Fastlock Hopping...\n");

   while (1) {

     // --- 1. Custom IP Group B-ലേക്ക് മാറാൻ കാത്തിരിക്കുക ---
     do {
       flag_value = Xil_In32(AXI_GPIO_BASE_ADDRESS) & 0x01;
     } while (flag_value == 0);

     // Group A സ്ലോട്ടുകളിലേക്ക് Weight അനുസരിച്ച് പുതിയ ഡാറ്റ നൽകുന്നു
     for (int slot = 0; slot < 4; slot++) {

       int profile_idx = get_weighted_profile();

       ad9361_tx_fastlock_load(ad9361_phy, slot,
                               fastlock_saved_data[profile_idx]);
     }

     // --- 2. Custom IP വീണ്ടും Group A-ലേക്ക് മാറാൻ കാത്തിരിക്കുക ---
     do {
       flag_value = Xil_In32(AXI_GPIO_BASE_ADDRESS) & 0x01;
     } while (flag_value != 0);

     // Group B സ്ലോട്ടുകളിലേക്ക് Weight അനുസരിച്ച് പുതിയ ഡാറ്റ നൽകുന്നു
     for (int slot = 4; slot < 8; slot++) {

       int profile_idx = get_weighted_profile();

       ad9361_tx_fastlock_load(ad9361_phy, slot,
                               fastlock_saved_data[profile_idx]);
     }

     loop_count++;
   }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  no_os_mdelay(10000);
  printf("DMA_EXAMPLE: address=%#lx samples=%lu channels=%u bits=%lu\n",
         (uintptr_t)dac_buffer, NO_OS_ARRAY_SIZE(dac_buffer),
         rx_adc_init.num_channels, 8 * sizeof(dac_buffer[0]));

#endif
#ifdef FMCOMMS5
  struct axi_dma_transfer read_transfer = {
      // Number of bytes to write/read
      .size = samples * AD9361_ADC_DAC_BYTES_PER_SAMPLE *
              ad9361_phy->rx_adc->num_channels,
      // Transfer done flag
      .transfer_done = 0,
      // Signal transfer mode
      .cyclic = NO,
      // Address of data source
      .src_addr = 0,
      // Address of data destination
      .dest_addr = (uintptr_t)ADC_DDR_BASEADDR};

  /* Read the data from the ADC DMA. */
  axi_dmac_transfer_start(rx_dmac, &read_transfer);

  /* Wait until transfer finishes */
  status = axi_dmac_transfer_wait_completion(rx_dmac, 500);
  if (status < 0)
    return status;
#else
  struct axi_dma_transfer read_transfer = {// Number of bytes to write/read
                                           .size = sizeof(adc_buffer),
                                           // Transfer done flag
                                           .transfer_done = 0,
                                           // Signal transfer mode
                                           .cyclic = NO,
                                           // Address of data source
                                           .src_addr = 0,
                                           // Address of data destination
                                           .dest_addr = (uintptr_t)adc_buffer};
  usleep(14);
  /* Read the data from the ADC DMA. */
  axi_dmac_transfer_start(rx_dmac, &read_transfer);

  /* Wait until transfer finishes */
  status = axi_dmac_transfer_wait_completion(rx_dmac, 500);
  if (status < 0)
    return status;
#endif
#ifdef XILINX_PLATFORM
#ifdef FMCOMMS5
  Xil_DCacheInvalidateRange((uintptr_t)ADC_DDR_BASEADDR,
                            samples * AD9361_ADC_DAC_BYTES_PER_SAMPLE *
                                ad9361_phy->rx_adc->num_channels);
  printf("DMA_EXAMPLE: address=%#x samples=%lu channels=%u bits=%u\n",
         (uintptr_t)ADC_DDR_BASEADDR,
         read_transfer.size / AD9361_ADC_DAC_BYTES_PER_SAMPLE,
         rx_adc_init.num_channels, 8 * sizeof(adc_buffer[0]));
#else
  Xil_DCacheInvalidateRange((uintptr_t)adc_buffer, sizeof(adc_buffer));

  /* --- UPDATED SECTION: PRINTING I/Q VALUES --- */
  printf("\n--- Printing first 200 I/Q Samples ---\n");
  for (uint32_t i = 0; i < 200; i++) {
    /* Interleaved format: [I0, Q0, I1, Q1, ...]
       Cast to int16_t to correctly display negative numbers */
    int16_t i_val = (int16_t)adc_buffer[2 * i];
    int16_t q_val = (int16_t)adc_buffer[2 * i + 1];

    printf("S[%lu]: I=%d, Q=%d\n", i, i_val, q_val);
  }
  printf("--- End of I/Q Data ---\n\n");
  /* ------------------------------------------- */

  printf("DMA_EXAMPLE: address=%#lx samples=%lu channels=%u bits=%lu\n",
         (uintptr_t)adc_buffer, NO_OS_ARRAY_SIZE(adc_buffer),
         rx_adc_init.num_channels, 8 * sizeof(adc_buffer[0]));
#endif
#endif
#endif
#endif

#ifdef IIO_SUPPORT
#ifdef SYSID_BASEADDR
  struct axi_sysid *sysid_core;
  char *name = NULL;
  struct axi_sysid_init_param sysid_init = {
      .base = SYSID_BASEADDR,
  };
#endif

  /**
   * iio application configurations.
   */
  struct xil_uart_init_param platform_uart_init_par = {
#ifdef XPAR_XUARTLITE_NUM_INSTANCES
      .type = UART_PL,
#else
      .type = UART_PS, .irq_id = UART_IRQ_ID
#endif
  };

  struct no_os_uart_init_param iio_uart_ip = {.device_id = UART_DEVICE_ID,
                                              .irq_id = UART_IRQ_ID,
                                              .baud_rate = UART_BAUDRATE,
                                              .size = NO_OS_UART_CS_8,
                                              .parity = NO_OS_UART_PAR_NO,
                                              .stop = NO_OS_UART_STOP_1_BIT,
                                              .extra = &platform_uart_init_par,
                                              .platform_ops = &xil_uart_ops};

#ifdef SYSID_BASEADDR
  status = axi_sysid_init(&sysid_core, &sysid_init);
  if (status)
    return status;
  ;

  name = axi_sysid_get_fpga_board(sysid_core);
  if (!strcmp("zed", name))
    iio_uart_ip.baud_rate = 115200;

  status = axi_sysid_remove(sysid_core);
  if (status)
    return status;
#endif

  struct iio_app_desc *app;
  struct iio_app_init_param app_init_param = {0};

  /**
   * iio axi adc configurations.
   */
  struct iio_axi_adc_init_param iio_axi_adc_init_par;
#ifdef FMCOMMS5
  struct iio_axi_adc_init_param iio_axi_adc_b_init_par;
#endif

  /**
   * iio axi dac configurations.
   */
  struct iio_axi_dac_init_param iio_axi_dac_init_par;
#ifdef FMCOMMS5
  struct iio_axi_dac_init_param iio_axi_dac_b_init_par;
#endif

  /**
   * iio ad9361 configurations.
   */
  struct iio_ad9361_init_param iio_ad9361_init_param;
#ifdef FMCOMMS5
  struct iio_ad9361_init_param iio_ad9361_b_init_param;
#endif

  /**
   * iio instance descriptor.
   */
  struct iio_axi_adc_desc *iio_axi_adc_desc;
#ifdef FMCOMMS5
  struct iio_axi_adc_desc *iio_axi_adc_b_desc;
#endif

  /**
   * iio instance descriptor.
   */
  struct iio_axi_dac_desc *iio_axi_dac_desc;
#ifdef FMCOMMS5
  struct iio_axi_dac_desc *iio_axi_dac_b_desc;
#endif

  /**
   * iio ad9361 instance descriptor.
   */
  struct iio_ad9361_desc *iio_ad9361_desc;
#ifdef FMCOMMS5
  struct iio_ad9361_desc *iio_ad9361_b_desc;
#endif

  /**
   * iio devices corresponding to every device.
   */
  struct iio_device *adc_dev_desc, *dac_dev_desc, *ad9361_dev_desc;
#ifdef FMCOMMS5
  struct iio_device *adc_b_dev_desc, *dac_b_dev_desc, *ad9361_b_dev_desc;
#endif

  status = axi_dmac_init(&tx_dmac, &tx_dmac_init);
  if (status < 0)
    return status;

  iio_axi_adc_init_par = (struct iio_axi_adc_init_param){
      .rx_adc = ad9361_phy->rx_adc,
      .rx_dmac = rx_dmac,
#ifndef PLATFORM_MB
      .dcache_invalidate_range =
          (void (*)(uint32_t, uint32_t))Xil_DCacheInvalidateRange
#endif
  };

  status = iio_axi_adc_init(&iio_axi_adc_desc, &iio_axi_adc_init_par);
  if (status < 0)
    return status;
  iio_axi_adc_get_dev_descriptor(iio_axi_adc_desc, &adc_dev_desc);

  struct iio_data_buffer read_buff = {
      .buff = (void *)ADC_DDR_BASEADDR,
      .size = 0xFFFFFFFF,
  };

#ifdef FMCOMMS5
  iio_axi_adc_b_init_par = (struct iio_axi_adc_init_param){
      .rx_adc = ad9361_phy_b->rx_adc,
  };

  status = iio_axi_adc_init(&iio_axi_adc_b_desc, &iio_axi_adc_b_init_par);
  if (status < 0)
    return status;
  iio_axi_adc_get_dev_descriptor(iio_axi_adc_b_desc, &adc_b_dev_desc);
#endif

  iio_axi_dac_init_par = (struct iio_axi_dac_init_param){
      .tx_dac = ad9361_phy->tx_dac,
      .tx_dmac = tx_dmac,
#ifndef PLATFORM_MB
      .dcache_flush_range = (void (*)(uint32_t, uint32_t))Xil_DCacheFlushRange,
#endif
  };

  status = iio_axi_dac_init(&iio_axi_dac_desc, &iio_axi_dac_init_par);
  if (status < 0)
    return status;
  iio_axi_dac_get_dev_descriptor(iio_axi_dac_desc, &dac_dev_desc);

  struct iio_data_buffer write_buff = {
      .buff = (void *)DAC_DDR_BASEADDR,
      .size = 0xFFFFFFFF,
  };

#ifdef FMCOMMS5
  iio_axi_dac_b_init_par = (struct iio_axi_dac_init_param){
      .tx_dac = ad9361_phy_b->tx_dac,
  };

  status = iio_axi_dac_init(&iio_axi_dac_b_desc, &iio_axi_dac_b_init_par);
  if (status < 0)
    return status;
  iio_axi_dac_get_dev_descriptor(iio_axi_dac_b_desc, &dac_b_dev_desc);
#endif

  iio_ad9361_init_param = (struct iio_ad9361_init_param){
      .ad9361_phy = ad9361_phy,
  };

  status = iio_ad9361_init(&iio_ad9361_desc, &iio_ad9361_init_param);
  if (status < 0)
    return status;
  iio_ad9361_get_dev_descriptor(iio_ad9361_desc, &ad9361_dev_desc);

#ifdef FMCOMMS5
  iio_ad9361_b_init_param = (struct iio_ad9361_init_param){
      .ad9361_phy = ad9361_phy_b,
  };

  status = iio_ad9361_init(&iio_ad9361_b_desc, &iio_ad9361_b_init_param);
  if (status < 0)
    return status;
  iio_ad9361_get_dev_descriptor(iio_ad9361_b_desc, &ad9361_b_dev_desc);
#endif

  struct iio_app_device devices[] = {
      IIO_APP_DEVICE("cf-ad9361-lpc", iio_axi_adc_desc, adc_dev_desc,
                     &read_buff, NULL, NULL),
      IIO_APP_DEVICE("cf-ad9361-dds-core-lpc", iio_axi_dac_desc, dac_dev_desc,
                     NULL, &write_buff, NULL),
      IIO_APP_DEVICE("ad9361-phy", ad9361_phy, ad9361_dev_desc, NULL, NULL,
                     NULL),
#ifdef FMCOMMS5
      IIO_APP_DEVICE("cf-ad9361-B", iio_axi_adc_b_desc, adc_b_dev_desc,
                     &read_buff, NULL, NULL),
      IIO_APP_DEVICE("cf-ad9361-dds-core-B", iio_axi_dac_b_desc, dac_b_dev_desc,
                     NULL, &write_buff, NULL),
      IIO_APP_DEVICE("ad9361-phy-B", ad9361_phy_b, ad9361_b_dev_desc, NULL,
                     NULL, NULL)
#endif
  };

  app_init_param.devices = devices;
  app_init_param.nb_devices = NO_OS_ARRAY_SIZE(devices);
  app_init_param.uart_init_params = iio_uart_ip;

  status = iio_app_init(&app, app_init_param);
  if (status)
    return status;

  iio_app_run(app);

#endif // IIO_SUPPORT

  printf("Done.\n");

#ifdef TDD_SWITCH_STATE_EXAMPLE
  uint32_t ensm_mode;
  struct no_os_gpio_init_param gpio_init = {.platform_ops = GPIO_OPS,
                                            .extra = GPIO_PARAM};
  struct no_os_gpio_desc *gpio_enable_pin;
  struct no_os_gpio_desc *gpio_txnrx_pin;
  if (!ad9361_phy->pdata->fdd) {
    if (ad9361_phy->pdata->ensm_pin_ctrl) {
      gpio_init.number = GPIO_ENABLE_PIN;
      status = no_os_gpio_get(&gpio_enable_pin, &gpio_init);
      if (status != 0) {
        printf("no_os_gpio_get() error: %" PRIi32 "\n", status);
        return status;
      }
      no_os_gpio_direction_output(gpio_enable_pin, 1);
      gpio_init.number = GPIO_TXNRX_PIN;
      status = no_os_gpio_get(&gpio_txnrx_pin, &gpio_init);
      if (status != 0) {
        printf("no_os_gpio_get() error: %" PRIi32 "\n", status);
        return status;
      }
      no_os_gpio_direction_output(gpio_txnrx_pin, 0);
      no_os_udelay(10);
      ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
      printf("TXNRX control - Alert: %s\n",
             ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
      no_os_mdelay(1000);

      if (ad9361_phy->pdata->ensm_pin_pulse_mode) {
        while (1) {
          no_os_gpio_set_value(gpio_txnrx_pin, 0);
          no_os_udelay(10);
          no_os_gpio_set_value(gpio_enable_pin, 1);
          no_os_udelay(10);
          no_os_gpio_set_value(gpio_enable_pin, 0);
          ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
          printf("TXNRX Pulse control - RX: %s\n",
                 ensm_mode == ENSM_MODE_RX ? "OK" : "Error");
          no_os_mdelay(1000);

          no_os_gpio_set_value(gpio_enable_pin, 1);
          no_os_udelay(10);
          no_os_gpio_set_value(gpio_enable_pin, 0);
          ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
          printf("TXNRX Pulse control - Alert: %s\n",
                 ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
          no_os_mdelay(1000);

          no_os_gpio_set_value(gpio_txnrx_pin, 1);
          no_os_udelay(10);
          no_os_gpio_set_value(gpio_enable_pin, 1);
          no_os_udelay(10);
          no_os_gpio_set_value(gpio_enable_pin, 0);
          ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
          printf("TXNRX Pulse control - TX: %s\n",
                 ensm_mode == ENSM_MODE_TX ? "OK" : "Error");
          no_os_mdelay(1000);

          no_os_gpio_set_value(gpio_enable_pin, 1);
          no_os_udelay(10);
          no_os_gpio_set_value(gpio_enable_pin, 0);
          ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
          printf("TXNRX Pulse control - Alert: %s\n",
                 ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
          no_os_mdelay(1000);
        }
      } else {
        while (1) {
          no_os_gpio_set_value(gpio_txnrx_pin, 0);
          no_os_udelay(10);
          no_os_gpio_set_value(gpio_enable_pin, 1);
          no_os_udelay(10);
          ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
          printf("TXNRX control - RX: %s\n",
                 ensm_mode == ENSM_MODE_RX ? "OK" : "Error");
          no_os_mdelay(1000);

          no_os_gpio_set_value(gpio_enable_pin, 0);
          no_os_udelay(10);
          ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
          printf("TXNRX control - Alert: %s\n",
                 ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
          no_os_mdelay(1000);

          no_os_gpio_set_value(gpio_txnrx_pin, 1);
          no_os_udelay(10);
          no_os_gpio_set_value(gpio_enable_pin, 1);
          no_os_udelay(10);
          ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
          printf("TXNRX control - TX: %s\n",
                 ensm_mode == ENSM_MODE_TX ? "OK" : "Error");
          no_os_mdelay(1000);

          no_os_gpio_set_value(gpio_enable_pin, 0);
          no_os_udelay(10);
          ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
          printf("TXNRX control - Alert: %s\n",
                 ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
          no_os_mdelay(1000);
        }
      }
    } else {
      while (1) {
        ad9361_set_en_state_machine_mode(ad9361_phy, ENSM_MODE_RX);
        ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
        printf("SPI control - RX: %s\n",
               ensm_mode == ENSM_MODE_RX ? "OK" : "Error");
        no_os_mdelay(1000);

        ad9361_set_en_state_machine_mode(ad9361_phy, ENSM_MODE_ALERT);
        ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
        printf("SPI control - Alert: %s\n",
               ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
        no_os_mdelay(1000);

        ad9361_set_en_state_machine_mode(ad9361_phy, ENSM_MODE_TX);
        ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
        printf("SPI control - TX: %s\n",
               ensm_mode == ENSM_MODE_TX ? "OK" : "Error");
        no_os_mdelay(1000);

        ad9361_set_en_state_machine_mode(ad9361_phy, ENSM_MODE_ALERT);
        ad9361_get_en_state_machine_mode(ad9361_phy, &ensm_mode);
        printf("SPI control - Alert: %s\n",
               ensm_mode == ENSM_MODE_ALERT ? "OK" : "Error");
        no_os_mdelay(1000);
      }
    }
  }
#endif

  ad9361_remove(ad9361_phy);
#ifdef FMCOMMS5
  ad9361_remove(ad9361_phy_b);
#endif

#ifdef XILINX_PLATFORM
  Xil_DCacheDisable();
  Xil_ICacheDisable();
#endif

#ifdef ALTERA_PLATFORM
  if (altera_bridge_uninit()) {
    printf("Altera Bridge Uninit Error!\n");
    return -1;
  }
#endif

  return 0;
}
