/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/config.c
 *
 * Copyright (C) 2025  Marc CAPDEVILLE (F4JMZ)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "config.h"

const spi_bus_config_t spi_config = {
	.mosi_io_num = CONFIG_ESP32S3APRS_SPI_D_GPIO,
	.miso_io_num = CONFIG_ESP32S3APRS_SPI_Q_GPIO,
	.sclk_io_num = CONFIG_ESP32S3APRS_SPI_CLK_GPIO,
	.max_transfer_sz = CONFIG_ESP32S3APRS_SPI_MAX_TRANSFER_SIZE, // for ssd1680 ram transfere is max 176*296/8 bytes align(4)
	.flags = SPICOMMON_BUSFLAG_MASTER ,
#if CONFIG_SPI_MASTER_ISR_IN_IRAM
	.intr_flags = ESP_INTR_FLAG_IRAM,
#endif
	.quadwp_io_num = -1,
	.quadhd_io_num = -1,
};

// Config for GDEY0213B74 B/W epd
const SSD1680_Config_t epd_config = {
	.xres = CONFIG_ESP32S3APRS_EPD_X_DEF,
	.yres = CONFIG_ESP32S3APRS_EPD_Y_DEF,
	.dpi = CONFIG_ESP32S3APRS_EPD_RESOLUTION,
	.driver_output_control = {
		.num_gate = CONFIG_ESP32S3APRS_EPD_Y_DEF,
		.tb = 0,
		.sm = 0,
		.gd = 0,
	},
	.border_waveform = {
		.transition = SSD1680_BORDER_WAVEFORM_TRANSITION_LUT1,
		.gs = 1,
		.zero = 0,
		.fix = SSD1680_BORDER_WAVEFORM_FIX_LEVEL_VSS,
		.select = SSD1680_BORDER_WAVEFORM_SELECT_GS,
	},
	.lut_end = {
		.option = SSD1680_LUT_END_OPTION_NORMAL,
	},

	.soft_start = {
		.phase = {{
			.min_off_time = SSD1680_BOOST_SS_MINOFF_8_4,
			.drive_strength = SSD1680_BOOST_SS_DRIVE_1,
			.one = 1,
		},{
			.min_off_time = SSD1680_BOOST_SS_MINOFF_6_3,
			.drive_strength = SSD1680_BOOST_SS_DRIVE_2,
			.one = 1,
		},{
			.min_off_time = SSD1680_BOOST_SS_MINOFF_3_9,
			.drive_strength = SSD1680_BOOST_SS_DRIVE_3,
			.one = 1,
		}},
		.duration = SSD1680_BOOST_SS_DURATION_P1_20 | SSD1680_BOOST_SS_DURATION_P2_20 | SSD1680_BOOST_SS_DURATION_P3_10,
	},

	.temp_sensor = {
		.sensor = SSD1680_TEMP_SENSOR_INTERNAL
	}, 
//	.load_sequence = {
//		.sequence = SSD1680_DISPLAY_UPDATE_SEQUENCE_LOAD_TEMP_LUT_1
//	},
	.update_sequence = {
		.sequence = SSD1680_DISPLAY_UPDATE_SEQUENCE_LOAD_TEMP_DISPLAY_1
	}
};

const SA8x8_config_t SA8x8_config = {
	.uart_num = CONFIG_ESP32S3APRS_RADIO_UART_NUM,
	.tx_pin = CONFIG_ESP32S3APRS_RADIO_TX_GPIO,
	.rx_pin = CONFIG_ESP32S3APRS_RADIO_RX_GPIO,
	.adc_pin = CONFIG_ESP32S3APRS_RADIO_ADC_GPIO,
	.dac_pin = CONFIG_ESP32S3APRS_RADIO_DAC_GPIO,
	.sq_pin = CONFIG_ESP32S3APRS_RADIO_SQ_GPIO,
	.hl_pin = CONFIG_ESP32S3APRS_RADIO_HL_GPIO,
	.pd_pin = CONFIG_ESP32S3APRS_RADIO_PD_GPIO,
	.ptt_pin = CONFIG_ESP32S3APRS_RADIO_PTT_GPIO,
	.sample_rate = CONFIG_ESP32S3APRS_RADIO_SAMPLE_RATE,
	.frame_len = CONFIG_ESP32S3APRS_RADIO_FRAME_LEN,
};

const AFSK_Config_t AFSK_Config = {
	.sample_rate = CONFIG_ESP32S3APRS_RADIO_SAMPLE_RATE,
	.baud_rate = 1200,
	.mark_freq = 1200,
	.space_freq = 2200
};

const esp_pm_config_t pm_config = {
	.max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
	.min_freq_mhz = CONFIG_XTAL_FREQ,
	.light_sleep_enable = true, // CONFIG_ESP32S3APRS_LIGHT_SLEEP,
};

const esp_vfs_spiffs_conf_t Spiffs_Config = {
	.base_path = CONFIG_ESP32S3APRS_SPIFFS_MOUNT_POINT,
	.partition_label = CONFIG_ESP32S3APRS_SPIFFS_PART_LABEL,
	.max_files = CONFIG_ESP32S3APRS_SPIFFS_MAX_FILES,
	.format_if_mount_failed = true
};

const uart_config_t console_config = {
	.baud_rate = CONFIG_ESP32S3APRS_CONSOLE_BAUD_RATE,
	.data_bits = UART_DATA_8_BITS,
	.parity = UART_PARITY_DISABLE,
	.stop_bits = UART_STOP_BITS_1,
	.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	.source_clk = UART_SCLK_XTAL,
};
