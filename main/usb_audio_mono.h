/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/usb_audio_mono.h
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

#ifndef _USB_AUDIO_MONO_H_
#define _USB_AUDIO_MONO_H_

#define AUDIO_TERM_TYPE_IN_RADIO_RECEIVER 0x0710
#define AUDIO_TERM_TYPE_OUT_RADIO_TRANSMITER 0x0711

enum {
	AUDIO_PATH_TRANSMITER=0,
	AUDIO_PATH_RECEIVER,
	AUDIO_PATH_MAX
};

enum {AUDIO_UNIT_INPUT=1,
	AUDIO_UNIT_FEATURE,
	AUDIO_UNIT_OUTPUT,
	AUDIO_UNIT_CLOCK,
	AUDIO_UNIT_MAX
};

// Unit numbers are arbitrary selected
// transmiter path
#define UAC2_ENTITY_TRANSMITER_INPUT_TERMINAL  (((AUDIO_PATH_TRANSMITER)<<4) | (AUDIO_UNIT_INPUT))
#define UAC2_ENTITY_TRANSMITER_FEATURE_UNIT    (((AUDIO_PATH_TRANSMITER)<<4) | (AUDIO_UNIT_FEATURE))
#define UAC2_ENTITY_TRANSMITER_OUTPUT_TERMINAL (((AUDIO_PATH_TRANSMITER)<<4) | (AUDIO_UNIT_OUTPUT))
#define UAC2_ENTITY_TRANSMITER_CLOCK           (((AUDIO_PATH_TRANSMITER)<<4) | (AUDIO_UNIT_CLOCK))

// receiver path
#define UAC2_ENTITY_RECEIVER_INPUT_TERMINAL    (((AUDIO_PATH_RECEIVER)<<4) | (AUDIO_UNIT_INPUT))
#define UAC2_ENTITY_RECEIVER_FEATURE_UNIT      (((AUDIO_PATH_RECEIVER)<<4) | (AUDIO_UNIT_FEATURE))
#define UAC2_ENTITY_RECEIVER_OUTPUT_TERMINAL   (((AUDIO_PATH_RECEIVER)<<4) | (AUDIO_UNIT_OUTPUT))
#define UAC2_ENTITY_RECEIVER_CLOCK             (((AUDIO_PATH_RECEIVER)<<4) | (AUDIO_UNIT_CLOCK))

#define TUD_AUDIO_DUPLEX_MONO_DESC_LEN (TUD_AUDIO_DESC_IAD_LEN\
		+ TUD_AUDIO_DESC_STD_AC_LEN\
		+ TUD_AUDIO_DESC_CS_AC_LEN\
		+ TUD_AUDIO_DESC_CLK_SRC_LEN\
		+ TUD_AUDIO_DESC_INPUT_TERM_LEN\
		+ TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN\
		+ TUD_AUDIO_DESC_OUTPUT_TERM_LEN\
		+ TUD_AUDIO_DESC_CLK_SRC_LEN\
		+ TUD_AUDIO_DESC_INPUT_TERM_LEN\
		+ TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN\
		+ TUD_AUDIO_DESC_OUTPUT_TERM_LEN\
		/* Interface 1, Alternate 0 */\
		+ TUD_AUDIO_DESC_STD_AS_INT_LEN\
		/* Interface 1, Alternate 1 */\
		+ TUD_AUDIO_DESC_STD_AS_INT_LEN\
		+ TUD_AUDIO_DESC_CS_AS_INT_LEN\
		+ TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN\
		+ TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN\
		+ TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN\
		+ TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_LEN\
		/* Interface 2, Alternate 0 */\
		+ TUD_AUDIO_DESC_STD_AS_INT_LEN\
		/* Interface 2, Alternate 1 */\
		+ TUD_AUDIO_DESC_STD_AS_INT_LEN\
		+ TUD_AUDIO_DESC_CS_AS_INT_LEN\
		+ TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN\
		+ TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN\
		+ TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN\
		)

#define TUD_AUDIO_DUPLEX_MONO_DESCRIPTOR(_stridx, itf_ct, itf_out, _epout, _epfb, itf_in, _epin) \
	/* Standard Interface Association Descriptor (IAD) */\
	TUD_AUDIO_DESC_IAD(/*_firstitf*/ itf_ct, /*_nitfs*/ 3, /*_stridx*/ 0x00),\
	/* Standard AC Interface Descriptor(4.7.1) */\
	TUD_AUDIO_DESC_STD_AC(/*_itfnum*/ itf_ct, /*_nEPs*/ 0x00, /*_stridx*/ _stridx),\
	/* Class-Specific AC Interface Header Descriptor(4.7.2) */\
	TUD_AUDIO_DESC_CS_AC(/*_bcdADC*/ 0x0200, /*_category*/ AUDIO_FUNC_OTHER, /*_totallen*/ TUD_AUDIO_DESC_CLK_SRC_LEN+TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN+TUD_AUDIO_DESC_INPUT_TERM_LEN+TUD_AUDIO_DESC_OUTPUT_TERM_LEN+TUD_AUDIO_DESC_CLK_SRC_LEN+TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN+TUD_AUDIO_DESC_INPUT_TERM_LEN+TUD_AUDIO_DESC_OUTPUT_TERM_LEN, /*_ctrl*/ (AUDIO_CTRL_R << AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS)),\
	\
	/* Clock Source Descriptor(4.7.2.1) */\
	TUD_AUDIO_DESC_CLK_SRC(/*_clkid*/ UAC2_ENTITY_TRANSMITER_CLOCK, /*_attr*/ AUDIO_CLOCK_SOURCE_ATT_INT_PRO_CLK, /*_ctrl*/ (AUDIO_CTRL_RW << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS) | (AUDIO_CTRL_R << AUDIO_CLOCK_SOURCE_CTRL_CLK_VAL_POS), /*_assocTerm*/ 0x00,  /*_stridx*/ 0x00),    \
	/* Input Terminal Descriptor(4.7.2.4) */\
	TUD_AUDIO_DESC_INPUT_TERM(/*_termid*/ UAC2_ENTITY_TRANSMITER_INPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x00, /*_clkid*/ UAC2_ENTITY_TRANSMITER_CLOCK, /*_nchannelslogical*/ 0x01, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_idxchannelnames*/ 0x00, /*_ctrl*/ 0x00, /*_stridx*/ 0x00),\
	/* Feature Unit Descriptor(4.7.2.8) */\
	TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL(/*_unitid*/ UAC2_ENTITY_TRANSMITER_FEATURE_UNIT, /*_srcid*/ UAC2_ENTITY_TRANSMITER_INPUT_TERMINAL, /*_ctrlch0master*/ (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS), /*_ctrlch1*/ 0, /*_stridx*/ 0x00),\
	/* Output Terminal Descriptor(4.7.2.5) */\
	TUD_AUDIO_DESC_OUTPUT_TERM(/*_termid*/ UAC2_ENTITY_TRANSMITER_OUTPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_OUT_RADIO_TRANSMITER, /*_assocTerm*/ 0x00, /*_srcid*/ UAC2_ENTITY_TRANSMITER_FEATURE_UNIT, /*_clkid*/ UAC2_ENTITY_TRANSMITER_CLOCK, /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),\
	\
	/* Clock Source Descriptor(4.7.2.1) */\
	TUD_AUDIO_DESC_CLK_SRC(/*_clkid*/ UAC2_ENTITY_RECEIVER_CLOCK, /*_attr*/ AUDIO_CLOCK_SOURCE_ATT_INT_PRO_CLK, /*_ctrl*/ (AUDIO_CTRL_RW << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS) | (AUDIO_CTRL_R << AUDIO_CLOCK_SOURCE_CTRL_CLK_VAL_POS), /*_assocTerm*/ 0x00,  /*_stridx*/ 0x00),    \
	/* Input Terminal Descriptor(4.7.2.4) */\
	TUD_AUDIO_DESC_INPUT_TERM(/*_termid*/ UAC2_ENTITY_RECEIVER_INPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_IN_RADIO_RECEIVER, /*_assocTerm*/ 0x00, /*_clkid*/ UAC2_ENTITY_RECEIVER_CLOCK, /*_nchannelslogical*/ 0x01, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_idxchannelnames*/ 0x00, /*_ctrl*/ 0x00, /*_stridx*/ 0x00),\
	/* Feature Unit Descriptor(4.7.2.8) */\
	TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL(/*_unitid*/ UAC2_ENTITY_RECEIVER_FEATURE_UNIT, /*_srcid*/ UAC2_ENTITY_RECEIVER_INPUT_TERMINAL, /*_ctrlch0master*/ (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS), /*_ctrlch1*/ 0, /*_stridx*/ 0x00),\
	/* Output Terminal Descriptor(4.7.2.5) */\
	TUD_AUDIO_DESC_OUTPUT_TERM(/*_termid*/ UAC2_ENTITY_RECEIVER_OUTPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x00, /*_srcid*/ UAC2_ENTITY_RECEIVER_FEATURE_UNIT, /*_clkid*/ UAC2_ENTITY_RECEIVER_CLOCK, /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),\
	\
	\
	/* Standard AS Interface Descriptor(4.9.1) */\
	/* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */\
	TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(itf_out), /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ _stridx+1),\
	/* Standard AS Interface Descriptor(4.9.1) */\
	/* Interface 1, Alternate 1 - alternate interface for data streaming */\
	TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(itf_out), /*_altset*/ 0x01, /*_nEPs*/ 0x02, /*_stridx*/ _stridx+1),\
	/* Class-Specific AS Interface Descriptor(4.9.2) */\
	TUD_AUDIO_DESC_CS_AS_INT(/*_termid*/ UAC2_ENTITY_TRANSMITER_INPUT_TERMINAL, /*_ctrl*/ AUDIO_CTRL_NONE, /*_formattype*/ AUDIO_FORMAT_TYPE_I, /*_formats*/ AUDIO_DATA_FORMAT_TYPE_I_PCM, /*_nchannelsphysical*/ CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_stridx*/ 0x00),\
	/* Type I Format Type Descriptor(2.3.1.6 - Audio Formats) */\
	TUD_AUDIO_DESC_TYPE_I_FORMAT(CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX, CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX),\
	/* Standard AS Isochronous Audio Data Endpoint Descriptor(4.10.1.1) */\
	TUD_AUDIO_DESC_STD_AS_ISO_EP(/*_ep*/ _epout, /*_attr*/ (uint8_t) (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA), /*_maxEPsize*/ TUD_AUDIO_EP_SIZE(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX), /*_interval*/ 0x01),\
	/* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.10.1.2) */\
	TUD_AUDIO_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, /*_ctrl*/ AUDIO_CTRL_NONE, /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, /*_lockdelay*/ 0x0000),\
	/* Standard AS Isochronous Feedback Endpoint Descriptor(4.10.2.1) */\
	TUD_AUDIO_DESC_STD_AS_ISO_FB_EP(/*_ep*/ _epfb, /*_interval*/ 1),\
	\
	\
	/* Standard AS Interface Descriptor(4.9.1) */\
	/* Interface 2, Alternate 0 - default alternate setting with 0 bandwidth */\
	TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(itf_in), /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ _stridx+2),\
	/* Standard AS Interface Descriptor(4.9.1) */\
	/* Interface 2, Alternate 1 - alternate interface for data streaming */\
	TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(itf_in), /*_altset*/ 0x01, /*_nEPs*/ 0x01, /*_stridx*/ _stridx+2),\
	/* Class-Specific AS Interface Descriptor(4.9.2) */\
	TUD_AUDIO_DESC_CS_AS_INT(/*_termid*/ UAC2_ENTITY_RECEIVER_OUTPUT_TERMINAL, /*_ctrl*/ AUDIO_CTRL_NONE, /*_formattype*/ AUDIO_FORMAT_TYPE_I, /*_formats*/ AUDIO_DATA_FORMAT_TYPE_I_PCM, /*_nchannelsphysical*/ CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_stridx*/ 0x00),\
	/* Type I Format Type Descriptor(2.3.1.6 - Audio Formats) */\
	TUD_AUDIO_DESC_TYPE_I_FORMAT(CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX, CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_TX),\
	/* Standard AS Isochronous Audio Data Endpoint Descriptor(4.10.1.1) */\
	TUD_AUDIO_DESC_STD_AS_ISO_EP(/*_ep*/ _epin, /*_attr*/ (uint8_t) (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA), /*_maxEPsize*/ TUD_AUDIO_EP_SIZE(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX), /*_interval*/ 0x01),\
	/* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.10.1.2) */\
	TUD_AUDIO_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, /*_ctrl*/ AUDIO_CTRL_NONE, AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, /*_lockdelay*/ 0x0000)\

#endif
