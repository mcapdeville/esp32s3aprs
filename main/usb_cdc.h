/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/usb_cdc.h
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

#ifndef _USB_CDC_H_
#define _USB_CDC_H_

#define USB_CDC_VFS_PATH	"/dev/ttyACM"

#if 0

// Length of template descriptor: 66 bytes
#define USB_CDC_SIMPLE_LEN  (8 +9+5+5 +9+7+7)

// CDC Descriptor Template
// Interface number, string index, EP notification address and size, EP data address (out, in) and size.
#define USB_CDC_SIMPLE(_itfnum, _stridx, _epdata, _epsize) \
  /* Interface Associate */\
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL, CDC_COMM_PROTOCOL_NONE, 0,\
  \
  /* CDC Control Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 0, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL, CDC_COMM_PROTOCOL_NONE, _stridx,\
  /* CDC Header */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_HEADER, U16_TO_U8S_LE(0x0120),\
  /* CDC Union */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_UNION, _itfnum, (uint8_t)((_itfnum) + 1), \
  \
  /* CDC Data Interface */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 0, 2, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epdata, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, ((_epdata) | 0x80), TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0\
  \
  /* CDC2 Data Interface */\
/*  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+2), 0, 2, TUSB_CLASS_CDC_DATA, 0, 0, 0,*/\
  /* Endpoint Out */\
/*  7, TUSB_DESC_ENDPOINT, _epcdc2, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,*/\
  /* Endpoint In */\
/*  7, TUSB_DESC_ENDPOINT, ((_epcdc2) | 0x80), TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0*/

#endif

int USB_CDC_Init(void);
int USB_CDC_register_itf(int itf);

#endif
