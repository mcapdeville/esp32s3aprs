/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/usb_cdc.c
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


#include <freertos/FreeRTOS.h>
#include <sys/fcntl.h>
#include <sys/lock.h>
#include <tusb.h>
#include <esp_vfs.h>
#include "usb.h"
#include "usb_cdc.h"
#include <errno.h>

#ifdef TAG
#undef TAG
#endif

#define TAG "USB_CDC"

#define USB_CDC_WR_TO	100
#define USB_CDC_RD_TO	portMAX_DELAY*portTICK_PERIOD_MS

static ssize_t USB_CDC_write(int fd, const void *data, size_t size);
static ssize_t USB_CDC_read(int fd, void *dst, size_t size);
static int USB_CDC_open(const char *path, int flags, int mode);
static int USB_CDC_close(int fd);

static const esp_vfs_t USB_CDC_vfs = {
    .flags = ESP_VFS_FLAG_DEFAULT,
    .open = &USB_CDC_open,
    .close = &USB_CDC_close,
    .read = &USB_CDC_read,
    .write = &USB_CDC_write,
};

static struct USB_CDC_itf {
	int itf;
	union {
		uint8_t rts:1;
		uint8_t dtr:1;
	};
	int readfd;
	int writefd;
	SemaphoreHandle_t read_lock;
	StaticSemaphore_t read_lock_buf;
	SemaphoreHandle_t write_lock;
	StaticSemaphore_t write_lock_buf;
	SemaphoreHandle_t rx_sem;
	StaticSemaphore_t rx_sem_buff;
	SemaphoreHandle_t tx_sem;
	StaticSemaphore_t tx_sem_buff;
} USB_CDC_itf[CFG_TUD_CDC];

int USB_CDC_Init(void) {

	int i,ret;

	for (i=0 ; i<CFG_TUD_CDC ; i++) {
		USB_CDC_itf[i].itf = -1;
		USB_CDC_itf[i].readfd = -1;
		USB_CDC_itf[i].writefd = -1;
	}

	ret = esp_vfs_register(USB_CDC_VFS_PATH, &USB_CDC_vfs, NULL);

	return ret;
}

int USB_CDC_register_itf(int itf) {
	int i;

	for (i=0 ; i<CFG_TUD_CDC ; i++)
		if (USB_CDC_itf[i].itf == itf)
			return -1;

	for (i=0 ; i<CFG_TUD_CDC ; i++) 
		if (USB_CDC_itf[i].itf == -1) {
			USB_CDC_itf[i].itf = itf;
			USB_CDC_itf[i].read_lock = xSemaphoreCreateMutexStatic(&USB_CDC_itf[i].read_lock_buf);
			USB_CDC_itf[i].write_lock = xSemaphoreCreateMutexStatic(&USB_CDC_itf[i].write_lock_buf);
			USB_CDC_itf[i].rx_sem = xSemaphoreCreateBinaryStatic(&USB_CDC_itf[i].rx_sem_buff);
			USB_CDC_itf[i].tx_sem = xSemaphoreCreateBinaryStatic(&USB_CDC_itf[i].tx_sem_buff);
			xSemaphoreTake(USB_CDC_itf[i].rx_sem,0);
			xSemaphoreGive(USB_CDC_itf[i].tx_sem);
			return i;
		}

	return -1;
}

static int USB_CDC_get_rd_fd(int fd) {
	int i;
	if (fd < 0)
		return -1;

	for (i=0 ; i<CFG_TUD_CDC ; i++)
		if (USB_CDC_itf[i].readfd == fd) {
			return i;
		}

	return -1;
}

static int USB_CDC_get_wr_fd(int fd) {
	int i;
	if (fd < 0)
		return -1;

	for (i=0 ; i<CFG_TUD_CDC ; i++)
		if (USB_CDC_itf[i].writefd == fd) {
			return i;
		}

	return -1;
}

static int USB_CDC_get_itf(int itf) {
	int i;
	if (itf < 0 || itf >= CFG_TUD_CDC)
		return -1;

	for (i=0 ; i<CFG_TUD_CDC ; i++)
		if (USB_CDC_itf[i].itf == itf) {
			return i;
		}

	return -1;
}

static ssize_t USB_CDC_write(int fd, const void *data, size_t size) {
	int num = USB_CDC_get_wr_fd(fd);
	int ret, total=0;

	if (num <0 || num >= CFG_TUD_CDC)
		return -1;

	xSemaphoreTake(USB_CDC_itf[num].write_lock, portMAX_DELAY);

	if (USB_CDC_itf[num].writefd != fd) {
		xSemaphoreGive(USB_CDC_itf[num].write_lock);
		ESP_LOGE(TAG,"writefd != fd");
		return -1;
	}

	if (!USB_CDC_itf[num].dtr) {
		// Discard datas
		xSemaphoreGive(USB_CDC_itf[num].write_lock);
		return size;
	}

	if (!tud_cdc_n_write_available(USB_CDC_itf[num].itf)) {
		tud_cdc_n_write_clear(USB_CDC_itf[num].itf);
		xSemaphoreGive(USB_CDC_itf[num].write_lock);
		errno = EWOULDBLOCK;
		return -1;
	}

	do {
		ret = tud_cdc_n_write(USB_CDC_itf[num].itf, data, size);
		tud_cdc_n_write_flush(USB_CDC_itf[num].itf);
/*		if (xSemaphoreTake(USB_CDC_itf[num].tx_sem, USB_CDC_WR_TO/portTICK_PERIOD_MS) != pdPASS) {
			tud_cdc_n_write_clear(USB_CDC_itf[num].itf);
			xSemaphoreGive(USB_CDC_itf[num].write_lock);
			errno = EIO;
			ESP_LOGE(TAG,"Write Timeout");
			return -1;
		}
*/		size -= ret;
		total += ret;
	}
	while (size && ret >0);

	xSemaphoreGive(USB_CDC_itf[num].write_lock);

	return total;
}

static ssize_t USB_CDC_read(int fd, void *dst, size_t size) {
	int num = USB_CDC_get_rd_fd(fd);
	int ret = 0;
	
	if (num <0 || num >= CFG_TUD_CDC) {
		ESP_LOGE(TAG,"Invalid fd in read");
		return -1;
	}

	xSemaphoreTake(USB_CDC_itf[num].read_lock, portMAX_DELAY);

	if (USB_CDC_itf[num].readfd != fd) {
		xSemaphoreGive(USB_CDC_itf[num].read_lock);
		ESP_LOGE(TAG,"readfd != fd");
		return -1;
	}
	
	if (!tud_cdc_n_available(USB_CDC_itf[num].itf)) {
		if (xSemaphoreTake(USB_CDC_itf[num].rx_sem, USB_CDC_RD_TO/portTICK_PERIOD_MS) != pdPASS) {
			xSemaphoreGive(USB_CDC_itf[num].read_lock);
			ESP_LOGE(TAG,"Read timeout");
			return -1;
		}
	}
	
	ret = tud_cdc_n_read(USB_CDC_itf[num].itf, dst, size);

	xSemaphoreGive(USB_CDC_itf[num].read_lock);

	return ret;
}

static int USB_CDC_open(const char *path, int flags, int mode) {
	int num;

	if (path[0] != '/')
		return -1;

	if (path[1]<'0' || path[1]>=(CFG_TUD_CDC+'0'))
		return -1;

	num = path[1]-'0';

	if (num <0 || num >= CFG_TUD_CDC)
		return -1;

	if ((flags & O_ACCMODE) == O_RDWR) {
			USB_CDC_itf[num].writefd = num;
			USB_CDC_itf[num].readfd = num;
			tud_cdc_n_write_clear(USB_CDC_itf[num].itf);
			xSemaphoreTake(USB_CDC_itf[num].rx_sem, 0);
			xSemaphoreGive(USB_CDC_itf[num].tx_sem);
			return USB_CDC_itf[num].writefd;
	}

	if ((flags & O_ACCMODE) == O_WRONLY) {
			USB_CDC_itf[num].writefd = (num<<1) +3;
			tud_cdc_n_write_clear(USB_CDC_itf[num].itf);
			xSemaphoreGive(USB_CDC_itf[num].tx_sem);
			return USB_CDC_itf[num].writefd;
	}

	if ((flags & O_ACCMODE) == O_RDONLY) {
			USB_CDC_itf[num].readfd = (num<<1) + 2;
			xSemaphoreTake(USB_CDC_itf[num].rx_sem, 0);
			return USB_CDC_itf[num].readfd;
	}

	return num;
}

static int USB_CDC_close(int fd) {
	int num;
	int ret = -1;

	if (fd <0 || fd >= CFG_TUD_CDC)
		return -1;

	num = USB_CDC_get_rd_fd(fd);
	if (num >=0 && num<CFG_TUD_CDC) {
		if (USB_CDC_itf[num].readfd == fd) {
			xSemaphoreGive(USB_CDC_itf[num].rx_sem);
			xSemaphoreTake(USB_CDC_itf[num].read_lock, portMAX_DELAY);
			USB_CDC_itf[num].readfd = -1;
			xSemaphoreGive(USB_CDC_itf[num].read_lock);
			ret = 0;
		}
	}

	num = USB_CDC_get_wr_fd(fd);
	if (num >=0 && num<CFG_TUD_CDC) {
		if (USB_CDC_itf[num].writefd == fd) {
			xSemaphoreGive(USB_CDC_itf[num].tx_sem);
			xSemaphoreTake(USB_CDC_itf[num].write_lock, portMAX_DELAY);
			USB_CDC_itf[num].writefd = -1;
			xSemaphoreGive(USB_CDC_itf[num].write_lock);
			ret = 0;
		}
	}
	
	return ret;
}

// Invoked when received new data
void tud_cdc_rx_cb(uint8_t Itf) {
	int num = USB_CDC_get_itf(Itf);
	if (num <0 || num >= CFG_TUD_CDC) {
		ESP_LOGE(TAG, "Error geting itf %d in rx_cb", Itf);
		return;
	}

	xSemaphoreGive(USB_CDC_itf[num].rx_sem);
}

// Invoked when a TX is complete and therefore space becomes available in TX buffer
void tud_cdc_tx_complete_cb(uint8_t Itf) {
	int num = USB_CDC_get_itf(Itf);
	if (num <0 || num >= CFG_TUD_CDC) {
		ESP_LOGE(TAG, "Error geting itf %d in tx_cb", Itf);
		return;
	}

	xSemaphoreGive(USB_CDC_itf[num].tx_sem);
}

// Invoked when line state DTR & RTS are changed via SET_CONTROL_LINE_STATE
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
	int num = USB_CDC_get_itf(itf);
	if (num <0 || num >= CFG_TUD_CDC)
		return;

	USB_CDC_itf[num].dtr = dtr;
	USB_CDC_itf[num].rts = rts;
}


