/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * ssd1680_def.h
 *
 * Copyright (C) 2025  Marc CAPDEVILLE F4JMZ
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

#ifndef _SSD1680_DEF_H_
#define _SSD1680_DEF_H_

// Gate driver output control
#define SSD1680_CMD_DRIVER_OUTPUT_CTRL 0x01
struct __attribute__((packed)) SSD1680_Driver_Output_Ctrl_S {
	uint16_t num_gate; // 1-296
	struct {
		uint8_t tb:1;
		uint8_t sm:1;
		uint8_t gd:1;
	};
};

// Gate Drving voltage control
#define SSD1680_CMD_GATE_VOLTAGE_CTRL 0x03
enum __attribute__((packed)) SSD1680_VGH_E {
	SSD1680_VGH_DEFAULT = 0x00,
	SSD1680_VGH_10_0 = 0x03,
	SSD1680_VGH_10_5,
	SSD1680_VGH_11_0,
	SSD1680_VGH_11_5,
	SSD1680_VGH_12_0,
	SSD1680_VGH_12_5,
	SSD1680_VGH_13_0,
	SSD1680_VGH_13_5,
	SSD1680_VGH_14_0,
	SSD1680_VGH_14_5,
	SSD1680_VGH_15_0,
	SSD1680_VGH_15_5,
	SSD1680_VGH_16_0,
	SSD1680_VGH_16_5,
	SSD1680_VGH_17_0,
	SSD1680_VGH_17_5,
	SSD1680_VGH_18_0,
	SSD1680_VGH_18_5,
	SSD1680_VGH_19_0,
	SSD1680_VGH_19_5,
	SSD1680_VGH_20_0
};

struct __attribute__((packed)) SSD1680_Gate_Voltage_Ctrl_S {
	enum SSD1680_VGH_E vgh;
};

// Source driving voltage control
#define SSD1680_CMD_SOURCE_VOLTAGE_CTRL	0x04
enum __attribute__((packed)) SSD1680_VSH_E {
	SSD1680_VSH_9_0 = 0x23,
	SSD1680_VSH_9_2,
	SSD1680_VSH_9_4,
	SSD1680_VSH_9_6,
	SSD1680_VSH_9_8,
	SSD1680_VSH_10_0,
	SSD1680_VSH_10_2,
	SSD1680_VSH_10_4,
	SSD1680_VSH_10_6,
	SSD1680_VSH_10_8,
	SSD1680_VSH_11_0,
	SSD1680_VSH_11_2,
	SSD1680_VSH_11_4,
	SSD1680_VSH_11_6,
	SSD1680_VSH_11_8,
	SSD1680_VSH_12_0,
	SSD1680_VSH_12_2,
	SSD1680_VSH_12_4,
	SSD1680_VSH_12_6,
	SSD1680_VSH_12_8,
	SSD1680_VSH_13_0,
	SSD1680_VSH_13_2,
	SSD1680_VSH_13_4,
	SSD1680_VSH_13_6,
	SSD1680_VSH_13_8,
	SSD1680_VSH_14_0,
	SSD1680_VSH_14_2,
	SSD1680_VSH_14_4,
	SSD1680_VSH_14_6,
	SSD1680_VSH_14_8,
	SSD1680_VSH_15_0,
	SSD1680_VSH_15_2,
	SSD1680_VSH_15_4,
	SSD1680_VSH_15_6,
	SSD1680_VSH_15_8,
	SSD1680_VSH_16_0,
	SSD1680_VSH_16_2,
	SSD1680_VSH_16_4,
	SSD1680_VSH_16_6,
	SSD1680_VSH_16_8,
	SSD1680_VSH_17_0,
	SSD1680_VSH_2_4 = 0x8E,
	SSD1680_VSH_2_5,
	SSD1680_VSH_2_6,
	SSD1680_VSH_2_7,
	SSD1680_VSH_2_8,
	SSD1680_VSH_2_9,
	SSD1680_VSH_3_0,
	SSD1680_VSH_3_1,
	SSD1680_VSH_3_2,
	SSD1680_VSH_3_3,
	SSD1680_VSH_3_4,
	SSD1680_VSH_3_5,
	SSD1680_VSH_3_6,
	SSD1680_VSH_3_7,
	SSD1680_VSH_3_8,
	SSD1680_VSH_3_9,
	SSD1680_VSH_4_0,
	SSD1680_VSH_4_1,
	SSD1680_VSH_4_2,
	SSD1680_VSH_4_3,
	SSD1680_VSH_4_4,
	SSD1680_VSH_4_5,
	SSD1680_VSH_4_6,
	SSD1680_VSH_4_7,
	SSD1680_VSH_4_8,
	SSD1680_VSH_4_9,
	SSD1680_VSH_5_0,
	SSD1680_VSH_5_1,
	SSD1680_VSH_5_2,
	SSD1680_VSH_5_3,
	SSD1680_VSH_5_4,
	SSD1680_VSH_5_5,
	SSD1680_VSH_5_6,
	SSD1680_VSH_5_7,
	SSD1680_VSH_5_8,
	SSD1680_VSH_5_9,
	SSD1680_VSH_6_0,
	SSD1680_VSH_6_1,
	SSD1680_VSH_6_2,
	SSD1680_VSH_6_3,
	SSD1680_VSH_6_4,
	SSD1680_VSH_6_5,
	SSD1680_VSH_6_6,
	SSD1680_VSH_6_7,
	SSD1680_VSH_6_8,
	SSD1680_VSH_6_9,
	SSD1680_VSH_7_0,
	SSD1680_VSH_7_1,
	SSD1680_VSH_7_2,
	SSD1680_VSH_7_3,
	SSD1680_VSH_7_4,
	SSD1680_VSH_7_5,
	SSD1680_VSH_7_6,
	SSD1680_VSH_7_7,
	SSD1680_VSH_7_8,
	SSD1680_VSH_7_9,
	SSD1680_VSH_8_0,
	SSD1680_VSH_8_1,
	SSD1680_VSH_8_2,
	SSD1680_VSH_8_3,
	SSD1680_VSH_8_4,
	SSD1680_VSH_8_5,
	SSD1680_VSH_8_6,
	SSD1680_VSH_8_7,
	SSD1680_VSH_8_8
};

enum __attribute__((packed)) SSD1680_VSL_E {
	SSD1680_VSL_5_0 = 0x0A,
	SSD1680_VSL_5_5 = 0x0C,
	SSD1680_VSL_6_0 = 0x0E,
	SSD1680_VSL_6_5 = 0x10,
	SSD1680_VSL_7_0 = 0x12,
	SSD1680_VSL_7_5 = 0x14,
	SSD1680_VSL_8_0 = 0x16,
	SSD1680_VSL_8_5 = 0x18,
	SSD1680_VSL_9_0 = 0x1A,
	SSD1680_VSL_9_5 = 0x1C,
	SSD1680_VSL_10_0 = 0x1E,
	SSD1680_VSL_10_5 = 0x20,
	SSD1680_VSL_11_0 = 0x22,
	SSD1680_VSL_11_5 = 0x24,
	SSD1680_VSL_12_0 = 0x26,
	SSD1680_VSL_12_5 = 0x28,
	SSD1680_VSL_13_0 = 0x2A,
	SSD1680_VSL_13_5 = 0x2C,
	SSD1680_VSL_14_0 = 0x2E,
	SSD1680_VSL_14_5 = 0x30,
	SSD1680_VSL_15_0 = 0x32
};

struct __attribute__((packed)) SSD1680_Source_Voltage_Ctrl_S {
	enum SSD1680_VSH_E vsh1;
	enum SSD1680_VSH_E vsh2;
	enum SSD1680_VSL_E vsl;
};

// Initial code setting OTP program
#define SSD1680_CMD_ICS_PROGRAM	0x08

// Initial code setting Write register
#define SSD1680_CMD_ICS_WRITE 0x09
// Initial code setting Read register
#define SSD1680_CMD_ICS_READ 0x0A
struct __attribute__((packed)) SSD1680_Ics_Register {
	uint8_t	a;
	uint8_t b;
	uint8_t c;
	uint8_t d;
};

// Booster soft start control
#define SSD1680_CMD_BOOST_SS_CTRL 0x0C
enum __attribute__((packed)) SSD1680_Boost_SS_Min_Off_E {
	SSD1680_BOOST_SS_MINOFF_2_6 = 0x04,
	SSD1680_BOOST_SS_MINOFF_3_2,
	SSD1680_BOOST_SS_MINOFF_3_9,
	SSD1680_BOOST_SS_MINOFF_4_6,
	SSD1680_BOOST_SS_MINOFF_5_4,
	SSD1680_BOOST_SS_MINOFF_6_3,
	SSD1680_BOOST_SS_MINOFF_7_3,
	SSD1680_BOOST_SS_MINOFF_8_4,
	SSD1680_BOOST_SS_MINOFF_9_8,
	SSD1680_BOOST_SS_MINOFF_11_5,
	SSD1680_BOOST_SS_MINOFF_13_8,
	SSD1680_BOOST_SS_MINOFF_16_5
};

enum __attribute__((packed)) SSD1680_Boost_SS_Drive_E {
	SSD1680_BOOST_SS_DRIVE_1 = 0x00,
	SSD1680_BOOST_SS_DRIVE_2,
	SSD1680_BOOST_SS_DRIVE_3,
	SSD1680_BOOST_SS_DRIVE_4,
	SSD1680_BOOST_SS_DRIVE_5,
	SSD1680_BOOST_SS_DRIVE_6,
	SSD1680_BOOST_SS_DRIVE_7,
	SSD1680_BOOST_SS_DRIVE_8
};

enum __attribute__((packed)) SSD1680_Boost_SS_Duration_E {
	SSD1680_BOOST_SS_DURATION_P1_10 = 0x00,
	SSD1680_BOOST_SS_DURATION_P1_20 = 0x01,
	SSD1680_BOOST_SS_DURATION_P1_30 = 0x02,
	SSD1680_BOOST_SS_DURATION_P1_40 = 0x03,
	SSD1680_BOOST_SS_DURATION_P2_10 = 0x00,
	SSD1680_BOOST_SS_DURATION_P2_20 = 0x04,
	SSD1680_BOOST_SS_DURATION_P2_30 = 0x08,
	SSD1680_BOOST_SS_DURATION_P2_40 = 0x0C,
	SSD1680_BOOST_SS_DURATION_P3_10 = 0x00,
	SSD1680_BOOST_SS_DURATION_P3_20 = 0x10,
	SSD1680_BOOST_SS_DURATION_P3_30 = 0x20,
	SSD1680_BOOST_SS_DURATION_P3_40 = 0x30,
};

struct __attribute__((packed)) SSD1680_Boost_SS_Ctrl_S {
	struct {
		enum SSD1680_Boost_SS_Min_Off_E min_off_time:4;
		enum SSD1680_Boost_SS_Drive_E drive_strength:3;
		uint8_t one:1;
	} phase[3];
	enum SSD1680_Boost_SS_Duration_E duration;
};

// Deep sleep mode
#define SSD1680_CMD_DEEP_SLEEP 0x10
enum __attribute__((packed)) SSD1680_Deep_Sleep_E {
	SSD1680_DEEP_SLEEP_NORMAL = 0x00,
	SSD1680_DEEP_SLEEP_MODE_1 = 0x01,
	SSD1680_DEEP_SLEEP_MODE_2 = 0x03
};

struct __attribute__((packed)) SSD1680_Deep_Sleep_S {
	enum SSD1680_Deep_Sleep_E mode;
};

// Data entry mode setting
#define SSD1680_CMD_DATA_ENTRY_MODE 0x11
enum __attribute__((packed)) SSD1680_Data_Entry_Mode_E {
	SSD1680_ENTRY_MODE_DEC = 0,
	SSD1680_ENTRY_MODE_INC = 1
};

enum __attribute__((packed)) SSD1680_Data_Entry_Mode_Dir_E {
	SSD1680_ENTRY_MODE_DIR_X = 0,
	SSD1680_ENTRY_MODE_DIR_Y = 1,
};

struct __attribute__((packed)) SSD1680_Data_Entry_Mode_S {
	struct {
		enum SSD1680_Data_Entry_Mode_E x:1;
		enum SSD1680_Data_Entry_Mode_E y:1;
		enum SSD1680_Data_Entry_Mode_Dir_E dir:1;
	};
};

// Software reset
#define SSD1680_CMD_SW_RESET	0x12

// HV Ready detection
#define SSD1680_CMD_HV_READY_DETECT 0x14
struct __attribute__((packed)) SSD1680_HV_Ready_Detect_S {
	struct {
		int8_t loop:3;
		int8_t dummy:1;
		int8_t time:3;
	};
};

// VCI detection
#define SSD1680_CMD_VCI_DETECT 0x15
enum __attribute__((packed)) SSD1680_VCI_Detect_Level_E {
	SSD1680_VCI_Detect_Level_2_2 = 0x03,
	SSD1680_VCI_Detect_Level_2_3,
	SSD1680_VCI_Detect_Level_2_4,
	SSD1680_VCI_Detect_Level_2_5,
	SSD1680_VCI_Detect_Level_2_6
};

struct __attribute__((packed)) SSD1680_VCI_Detect_S {
	enum SSD1680_VCI_Detect_Level_E level;
};

// Temperature sensor control
#define SSD1680_CMD_TEMP_SENSOR_CTRL 0x18
enum __attribute__((packed)) SSD1680_Temp_Sensor_E {
	SSD1680_TEMP_SENSOR_EXTERNAL = 0x48,
	SSD1680_TEMP_SENSOR_INTERNAL = 0x80
};

struct __attribute__((packed)) SSD1680_Temp_Sensor_S {
	enum SSD1680_Temp_Sensor_E sensor;
};

// Temperature register write
#define SSD1680_CMD_TEMP_REGISTER_WRITE 0x1A
// Temperature register read
#define SSD1680_CMD_TEMP_REGISTER_READ 0x1B
struct __attribute__((packed)) SSD1680_Temp_Register_S {
	struct {
		uint16_t dummy:4;
		uint16_t temp:12;
	};
};

// Write to external temp sensor
#define SSD1680_CMD_EXT_SENSOR_WRITE 0x01C
enum __attribute__((packed)) SSD1680_Ext_Sensor_Write_E {
	SSD1680_EXT_SENSOR_WRITE_2_BYTES = 0x00,
	SSD1680_EXT_SENSOR_WRITE_3_BYTES,
	SSD1680_EXT_SENSOR_WRITE_4_BYTES,
	SSD1680_EXT_SENSOR_WRITE_1_BYTES
};

struct __attribute__((packed)) SSD1680_Ext_Sensor_Write_S {
	struct {
		uint8_t pointer:6;
		enum SSD1680_Ext_Sensor_Write_E len:2;
	};
	uint8_t Param1;
	uint8_t Param2;
};

#define SSD1680_CMD_MASTER_ACTIVATION 0x20

#define SSD1680_CMD_DISPLAY_UPDATE_CTRL_1 0x21
enum __attribute__((packed)) SSD1680_Display_Update_RAM_Opt_E {
	SSD1680_DISPLAY_UPDATE_RAM_NORMAL = 0x00,
	SSD1680_DISPLAY_UPDATE_RAM_BYPASS = 0x04,
	SSD1680_DISPLAY_UPDATE_RAM_INVERSE = 0x08
};

enum __attribute__((packed)) SSD1680_Display_Update_Source_Opt_E {
	SSD1680_DISPLAY_UPDATE_SOURCE_0_175 = 0x00,
	SSD1680_DISPLAY_UPDATE_SOURCE_8_167 = 0x80
};

struct __attribute__((packed)) SSD1680_Display_Update_Ctrl_1 {
	struct {
		enum SSD1680_Display_Update_RAM_Opt_E bw_ram_opt;
		enum SSD1680_Display_Update_RAM_Opt_E red_ram_opt;
	};
	enum SSD1680_Display_Update_Source_Opt_E source;
};

#define SSD1680_CMD_DISPLAY_UPDATE_CTRL_2 0x22
enum __attribute__((packed)) SSD1680_Display_Update_Sequence_E {
	SSD1680_DISPLAY_UPDATE_SEQUENCE_CLOCK_EN = 0x80,	// enable Clock
	SSD1680_DISPLAY_UPDATE_SEQUENCE_CLOCK_DIS = 0x01,	// Disable Clock
	SSD1680_DISPLAY_UPDATE_SEQUENCE_ANALOG_EN = 0xC0,	// Eneble Clock,Analog
	SSD1680_DISPLAY_UPDATE_SEQUENCE_ANALOG_DIS = 0x03,	// Disable Analog,Clock
	SSD1680_DISPLAY_UPDATE_SEQUENCE_LOAD_LUT_1 = 0x91,	// Enable Clock, Load LUT, Disable Clock
	SSD1680_DISPLAY_UPDATE_SEQUENCE_LOAD_LUT_2 = 0x99,	// Enable Clock, Load LUT, Disable Clock
	SSD1680_DISPLAY_UPDATE_SEQUENCE_LOAD_TEMP_LUT_1 = 0xB1, // Enable CLock, Load Temp, Load LUT, DISABLE_CLOCK
	SSD1680_DISPLAY_UPDATE_SEQUENCE_LOAD_TEMP_LUT_2 = 0xB9, // Enable CLock, Load Temp, Load LUT, DISABLE_CLOCK
	SSD1680_DISPLAY_UPDATE_SEQUENCE_DISPLAY_1 = 0xC7, // Enable Clock, Analog, Display, Disable Analog, Clock
	SSD1680_DISPLAY_UPDATE_SEQUENCE_DISPLAY_2 = 0xCF, // Enable Clock, Analog, Display, Disable Analog, Clock
	SSD1680_DISPLAY_UPDATE_SEQUENCE_LOAD_TEMP_DISPLAY_1 = 0xF7, // Enable Clock, Analog, Load Temp, Display, Disable Analog, Clock
	SSD1680_DISPLAY_UPDATE_SEQUENCE_LOAD_TEMP_DISPLAY_2 = 0xFF, // Enable Clock, Analog, Load Temp, Display, Disable Analog, Clock
};

struct __attribute__((packed)) SSD1680_Display_Update_Sequence_S {
	enum SSD1680_Display_Update_Sequence_E sequence;
};

// Write BW RAM
#define SSD1680_CMD_WRITE_BW_RAM 0x24

// Write Red RAM
#define SSD1680_CMD_WRITE_RED_RAM 0x26

// VCOM sense
#define SSD1680_CMD_VCOM_SENSE 0x28

// VCOM sense duration
#define SSD1680_CMD_VCOM_DURATION_SENSE 0x29
struct __attribute__((packed)) SSD1680_VCOM_Sense_Duration_S {
	struct {
		uint8_t duration:4;
		uint8_t four:4;
	};
};

// Program VCOM in OPT
#define SSD1680_CMD_PROGRAM_VCOM 0x2A

// Write Register for VCOM Control
#define SSD1680_CMD_WRITE_VCOM_REG_CTRL 0x2B
struct __attribute__((packed)) SSD1680_Write_VCOM_Reg_Ctrl {
	uint8_t x04;	// Must be filled with 0x04
	uint8_t x64;	// Must be filled with 0x64
};

// Write VCOM register
#define SSD1680_CMD_WRITE_VCOM_REG 0x2C
enum __attribute__((packed)) SSD1680_VCOM_Register_E {
	SSD1680_VCOM_REG_0_2 = 0x08,
	SSD1680_VCOM_REG_0_3 = 0xC,
	SSD1680_VCOM_REG_0_4 = 0x10,
	SSD1680_VCOM_REG_0_5 = 0x14,
	SSD1680_VCOM_REG_0_6 = 0x18,
	SSD1680_VCOM_REG_0_7 = 0x1C,
	SSD1680_VCOM_REG_0_8 = 0x20,
	SSD1680_VCOM_REG_0_9 = 0x24,
	SSD1680_VCOM_REG_1_0 = 0x28,
	SSD1680_VCOM_REG_1_1 = 0x2C,
	SSD1680_VCOM_REG_1_2 = 0x30,
	SSD1680_VCOM_REG_1_3 = 0x34,
	SSD1680_VCOM_REG_1_4 = 0x38,
	SSD1680_VCOM_REG_1_5 = 0x3C,
	SSD1680_VCOM_REG_1_6 = 0x40,
	SSD1680_VCOM_REG_1_7 = 0x44,
	SSD1680_VCOM_REG_1_8 = 0x48,
	SSD1680_VCOM_REG_1_9 = 0x4C,
	SSD1680_VCOM_REG_2_0 = 0x50,
	SSD1680_VCOM_REG_2_1 = 0x54,
	SSD1680_VCOM_REG_2_2 = 0x58,
	SSD1680_VCOM_REG_2_3 = 0x5C,
	SSD1680_VCOM_REG_2_4 = 0x60,
	SSD1680_VCOM_REG_2_5 = 0x64,
	SSD1680_VCOM_REG_2_6 = 0x68,
	SSD1680_VCOM_REG_2_7 = 0x6C,
	SSD1680_VCOM_REG_2_8 = 0x70,
	SSD1680_VCOM_REG_2_9 = 0x74,
	SSD1680_VCOM_REG_3_0 = 0x78
};

struct __attribute__((packed)) SSD1680_Write_VCOM_Reg_S {
	enum SSD1680_VCOM_Register_E vcom;
};

// OTP Register read for display option
#define SSD1680_CMD_OTP_READ_DISPLAY_OPTION 0x2D
enum __attribute__((packed)) SSD1680_VCOM_OTP_Selection_E {
	SSD1680_VCOM_OTP_SELECTION_NORMAL = 0x00,
	SSD1680_VCOM_OTP_SELECTION_SPARE = 0x80
};

enum __attribute__((packed)) SSD1680_Display_Mode_E {
	SSD1680_DISPLAY_MODE_1 = 0x00,
	SSD1680_DISPLAY_MODE_2= 0x01,
};

enum __attribute__((packed)) SSD1680_Display_Pingpong_E {
	SSD1680_DISPLAY_PINGPONG_DISABLE = 0x00,
	SSD1680_DISPLAY_PINGPONG_ENABLE = 0x01
};

struct __attribute__((packed)) SSD1680_OTP_Display_Option_S {
	enum SSD1680_VCOM_OTP_Selection_E vcom_selection;
	enum SSD1680_VCOM_Register_E vcom;
	union {
		uint8_t display_mode[5];
		struct {
			enum SSD1680_Display_Mode_E ws0:1;
			enum SSD1680_Display_Mode_E ws1:1;
			enum SSD1680_Display_Mode_E ws2:1;
			enum SSD1680_Display_Mode_E ws3:1;
			enum SSD1680_Display_Mode_E ws4:1;
			enum SSD1680_Display_Mode_E ws5:1;
			enum SSD1680_Display_Mode_E ws6:1;
			enum SSD1680_Display_Mode_E ws7:1;
			enum SSD1680_Display_Mode_E ws8:1;
			enum SSD1680_Display_Mode_E ws9:1;
			enum SSD1680_Display_Mode_E ws10:1;
			enum SSD1680_Display_Mode_E ws11:1;
			enum SSD1680_Display_Mode_E ws12:1;
			enum SSD1680_Display_Mode_E ws13:1;
			enum SSD1680_Display_Mode_E ws14:1;
			enum SSD1680_Display_Mode_E ws15:1;
			enum SSD1680_Display_Mode_E ws16:1;
			enum SSD1680_Display_Mode_E ws17:1;
			enum SSD1680_Display_Mode_E ws18:1;
			enum SSD1680_Display_Mode_E ws19:1;
			enum SSD1680_Display_Mode_E ws20:1;
			enum SSD1680_Display_Mode_E ws21:1;
			enum SSD1680_Display_Mode_E ws22:1;
			enum SSD1680_Display_Mode_E ws23:1;
			enum SSD1680_Display_Mode_E ws24:1;
			enum SSD1680_Display_Mode_E ws25:1;
			enum SSD1680_Display_Mode_E ws26:1;
			enum SSD1680_Display_Mode_E ws27:1;
			enum SSD1680_Display_Mode_E ws28:1;
			enum SSD1680_Display_Mode_E ws29:1;
			enum SSD1680_Display_Mode_E ws30:1;
			enum SSD1680_Display_Mode_E ws31:1;
			enum SSD1680_Display_Mode_E ws32:1;
			enum SSD1680_Display_Mode_E ws33:1;
			enum SSD1680_Display_Mode_E ws34:1;
			enum SSD1680_Display_Mode_E ws35:1;
		};
		struct {
			uint8_t dummy_1[3];
			enum SSD1680_Display_Pingpong_E dummy_2:6;
			enum SSD1680_Display_Pingpong_E pingpong:1;
		};
	};
	uint32_t module_id;
};

/* ================================= */
/* ================================= */

#define SSD1680_CMD_BORDER_WAVEFORM 0x3C
enum __attribute__((packed)) SSD1680_Border_Waveform_Transition_E {
	SSD1680_BORDER_WAVEFORM_TRANSITION_LUT0 = 0x00,
	SSD1680_BORDER_WAVEFORM_TRANSITION_LUT1,
	SSD1680_BORDER_WAVEFORM_TRANSITION_LUT2,
	SSD1680_BORDER_WAVEFORM_TRANSITION_LUT3
};

enum __attribute__((packed)) SSD1680_Border_Waveform_Fix_Level_E {
	SSD1680_BORDER_WAVEFORM_FIX_LEVEL_VSS = 0x00,
	SSD1680_BORDER_WAVEFORM_FIX_LEVEL_VSH1,
	SSD1680_BORDER_WAVEFORM_FIX_LEVEL_VSL,
	SSD1680_BORDER_WAVEFORM_FIX_LEVEL_VSH2
};

enum __attribute__((packed)) SSD1680_Border_Waveform_Select_E {
	SSD1680_BORDER_WAVEFORM_SELECT_GS = 0x00,
	SSD1680_BORDER_WAVEFORM_SELECT_FIX,
	SSD1680_BORDER_WAVEFORM_SELECT_VCOM,
	SSD1680_BORDER_WAVEFORM_SELECT_HIZ,
};

struct __attribute__((packed)) SSD1680_Border_Waveform_S {
	struct {
		enum SSD1680_Border_Waveform_Transition_E transition:2;
		uint8_t gs:1;
		uint8_t zero:1;
		enum SSD1680_Border_Waveform_Fix_Level_E fix:2;
		enum SSD1680_Border_Waveform_Select_E select:2;
	};
};

#define SSD1680_CMD_LUT_END_OPTION 0x3F
enum __attribute__((packed)) SSD1680_Lut_End_Option_E {
	SSD1680_LUT_END_OPTION_NORMAL = 0x22,
	SSD1680_LUT_END_OPTION_KEEP = 0x07,
};

struct __attribute__((packed)) SSD1680_Lut_End_Option_S {
	enum SSD1680_Lut_End_Option_E option;
};

/* ================================= */
/* ================================= */


#define SSD1680_CMD_SET_RAM_X_POS 0x44
struct __attribute__((packed)) SSD1680_Set_Ram_X_Pos_S {
	uint8_t start;
	uint8_t end;
};
#define SSD1680_CMD_SET_RAM_Y_POS 0x45
struct __attribute__((packed)) SSD1680_Set_Ram_Y_Pos_S {
	uint16_t start;
	uint16_t end;
};

#define SSD1680_CMD_SET_RAM_X_COUNTER 0x4E
struct __attribute__((packed)) SSD1680_Set_Ram_X_Counter_S {
	uint8_t counter;
};
#define SSD1680_CMD_SET_RAM_Y_COUNTER 0x4F
struct __attribute__((packed)) SSD1680_Set_Ram_Y_Counter_S {
	uint16_t counter;
};

#define SSD1680_CMD_NOP 0x7F

#endif
