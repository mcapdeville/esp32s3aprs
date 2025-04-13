/*
 * SPDX-License-Identifier: MIT
 *
 * ESP32s3APRS by F4JMZ
 *
 * main/micropython.c
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George
 * Copyright (c) 2025 Marc CAPDEVILLE (F4JMZ)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <errno.h>
#include <unistd.h>
#include <sys/fcntl.h>
#include <sys/poll.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "esp_psram.h"

#include "py/stackctrl.h"
#include "py/nlr.h"
// #include "py/compile.h"
#include "py/runtime.h"
#include "py/builtin.h"
#include "py/mperrno.h"
#include "py/persistentcode.h"
//#include "py/repl.h"
#include "py/gc.h"
#include "py/mphal.h"
#include "py/stream.h"
#include "shared/readline/readline.h"
#include "shared/runtime/pyexec.h"
#include "mpthreadport.h"
#include "modmachine.h"
#include "extmod/vfs.h"
#include "extmod/vfs_posix.h"

#include <py/obj.h>
#include <py/objstr.h>

#define TAG "MPY"

#if MICROPY_BLUETOOTH_NIMBLE
#include "extmod/modbluetooth.h"
#endif

// MicroPython runs as a task under FreeRTOS
#define MP_TASK_PRIORITY        (2)
#ifndef NDEBUG
#define MP_TASK_STACK_SIZE      4096+1024
#else
#define MP_TASK_STACK_SIZE      4096
#endif
#define MP_MIN_HEAP_SIZE		(32768)

// Set the margin for detecting stack overflow, depending on the CPU architecture.
#if CONFIG_IDF_TARGET_ESP32C3
#define MP_TASK_STACK_LIMIT_MARGIN (2048)
#else
#define MP_TASK_STACK_LIMIT_MARGIN (1024)
#endif

#define PATHLIST_SEP_CHAR ':'

#if MICROPY_EMIT_NATIVE
#warning " EMIT NATIVE"
#endif

#if MICROPY_PERISTENT_CODE_LOAD
#warning "PERSISTENT CODE LOAD"
#endif

TaskHandle_t mp_main_task_handle;
portMUX_TYPE mp_atomic_mux = portMUX_INITIALIZER_UNLOCKED;

#if MICROPY_EMIT_NATIVE || MICROPY_PERSISTENT_CODE_LOAD
typedef struct _native_code_node_t {
    struct _native_code_node_t *next;
    uint32_t data[];
} native_code_node_t;

static native_code_node_t *native_code_head = NULL;

static void esp_native_code_free_all(void);
#endif

// Check the ESP-IDF error code and raise an OSError if it's not ESP_OK.
#if MICROPY_ERROR_REPORTING <= MICROPY_ERROR_REPORTING_NORMAL
void check_esp_err_(esp_err_t code)
#else
void check_esp_err_(esp_err_t code, const char *func, const int line, const char *file)
#endif
{
    if (code != ESP_OK) {
        // map esp-idf error code to posix error code
        uint32_t pcode = -code;
        switch (code) {
            case ESP_ERR_NO_MEM:
                pcode = MP_ENOMEM;
                break;
            case ESP_ERR_TIMEOUT:
                pcode = MP_ETIMEDOUT;
                break;
            case ESP_ERR_NOT_SUPPORTED:
                pcode = MP_EOPNOTSUPP;
                break;
        }
        // construct string object
        mp_obj_str_t *o_str = m_new_obj_maybe(mp_obj_str_t);
        if (o_str == NULL) {
            mp_raise_OSError(pcode);
            return;
        }
        o_str->base.type = &mp_type_str;
        #if MICROPY_ERROR_REPORTING > MICROPY_ERROR_REPORTING_NORMAL
        char err_msg[64];
        esp_err_to_name_r(code, err_msg, sizeof(err_msg));
        vstr_t vstr;
        vstr_init(&vstr, 80);
        vstr_printf(&vstr, "0x%04X %s in function '%s' at line %d in file '%s'", code, err_msg, func, line, file);
        o_str->data = (const byte *)vstr_null_terminated_str(&vstr);
        #else
        o_str->data = (const byte *)esp_err_to_name(code); // esp_err_to_name ret's ptr to const str
        #endif
        o_str->len = strlen((char *)o_str->data);
        o_str->hash = qstr_compute_hash(o_str->data, o_str->len);
        // raise
        mp_obj_t args[2] = { MP_OBJ_NEW_SMALL_INT(pcode), MP_OBJ_FROM_PTR(o_str)};
        nlr_raise(mp_obj_exception_make_new(&mp_type_OSError, 2, 0, args));
    }
}

#if defined(MICROPY_INCLUDED_LIB_UTILS_INTERRUPT_CHAR_H) && !MICROPY_KBD_EXCEPTION

int mp_interrupt_char = -1;

void mp_hal_set_interrupt_char(int c) {
    mp_interrupt_char = c;
}
#endif

#if MICROPY_ENABLE_COMPILER && !(MICROPY_READER_POSIX || MICROPY_READER_VFS)

mp_lexer_t *mp_lexer_new_from_file(qstr filename) {
    return mp_const_none;
}
#if MICROPY_HELPER_LEXER_UNIX

mp_lexer_t *mp_lexer_new_from_fd(qstr filename, int fd, bool close_fd) {
	return mp_const_none;
}
#endif
#endif

int vprintf_null(const char *format, va_list ap) {
    // do nothing: this is used as a log target during raw repl mode
    return 0;
}

struct {
	TaskHandle_t handle;
	StackType_t	stack[(MP_TASK_STACK_SIZE)/sizeof(StackType_t)];
	StaticTask_t buffer;
	char console[PATH_MAX];
	FILE * console_in;
	FILE * console_out;
	int in_fd, out_fd;
} MP_Tasks;

void mp_task(void *pvParameter) {
    size_t mp_task_heap_size;
    void *mp_task_heap = NULL;

    volatile uint32_t sp = (uint32_t)esp_cpu_get_sp();

    portMUX_INITIALIZE(&mp_atomic_mux);
#if MICROPY_PY_THREAD
    mp_thread_init(pxTaskGetStackStart(NULL), MP_TASK_STACK_SIZE / sizeof(uintptr_t));
#endif

#if MICROPY_PY_MACHINE
    machine_init();
#endif

#if CONFIG_SPIRAM
    #if CONFIG_SPIRAM_USE_MALLOC
    // SPIRAM is issued using MALLOC, fallback to normal allocation rules
    mp_task_heap = NULL;
    #elif CONFIG_SOC_SPIRAM_SUPPORTED
    // Try to use the entire external SPIRAM directly for the heap
    size_t esp_spiram_size = esp_psram_get_size();
    if (esp_spiram_size > 0) {
        mp_task_heap = (void *)SOC_EXTRAM_DATA_HIGH - esp_spiram_size;
        mp_task_heap_size = esp_spiram_size;
		ESP_LOGI(TAG,"esp_psram size %u", mp_task_heap_size);
    } else {
		ESP_LOGD(TAG,"esp_psram not found");
	}
    #endif
#endif

    if (mp_task_heap == NULL) {
        // Allocate the uPy heap using malloc and get the largest available region,
        // limiting to 1/2 total available memory to leave memory for the OS.
        #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 1, 0)
        size_t heap_total = heap_caps_get_total_size(MALLOC_CAP_8BIT);
        #else
        multi_heap_info_t info;
        heap_caps_get_info(&info, MALLOC_CAP_8BIT);
        size_t heap_total = info.total_free_bytes + info.total_allocated_bytes;
        #endif
		do {
			mp_task_heap_size = MIN(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT), heap_total / 2);
			mp_task_heap = malloc(mp_task_heap_size);
		} while (!mp_task_heap && mp_task_heap_size > MP_MIN_HEAP_SIZE);

		if (mp_task_heap)
			ESP_LOGI(TAG, "Allocating %d bytes on system heap", mp_task_heap_size);
		else
			ESP_LOGE(TAG, "Not enough memory");
    }

	if (!mp_task_heap) {
		vTaskDelete(NULL);
	}

soft_reset:
	MP_Tasks.in_fd = 0;
	MP_Tasks.out_fd = 1;

	if (MP_Tasks.console[0]) {
		// Open the console
		if ((MP_Tasks.console_in = fopen(MP_Tasks.console, "r"))) {
			MP_Tasks.in_fd = fileno(MP_Tasks.console_in);
			//		stdin = console_in;
		} else {
			ESP_LOGE(TAG,"Can't open %s for reading (%d)", MP_Tasks.console,  errno);
		}
		if ((MP_Tasks.console_out = fopen(MP_Tasks.console, "w"))) {
			MP_Tasks.out_fd = fileno(MP_Tasks.console_out);
			//		stdout = console_out;
		} else {
			ESP_LOGE(TAG,"Can't open %s for writing (%d)", MP_Tasks.console,  errno);
		}
	}

    // initialise the stack pointer for the main thread
    mp_stack_set_top((void *)sp);
    mp_stack_set_limit(MP_TASK_STACK_SIZE - MP_TASK_STACK_LIMIT_MARGIN);
    gc_init(mp_task_heap, mp_task_heap + mp_task_heap_size);
    mp_init();
   
#if MICROPY_PY_SYS && !MICROPY_PY_SYS_PATH_ARGV_DEFAULTS
    const char * path = MICROPY_PY_SYS_PATH_DEFAULT;
    size_t path_num = 1; // [0] is for current dir (or base dir of the script)
						  // u_
    if (*path == PATHLIST_SEP_CHAR) {
        path_num++;
    }
    for (const char *p = path; p != NULL; p = strchr(p, PATHLIST_SEP_CHAR)) {
        path_num++;
        if (p != NULL) {
            p++;
        }
    }
    mp_sys_path = m_new_obj(mp_obj_list_t);
    mp_obj_list_init(MP_OBJ_TO_PTR(mp_sys_path), path_num);
    mp_obj_t *path_items;
    mp_obj_list_get(mp_sys_path, &path_num, &path_items);
    path_items[0] = MP_OBJ_NEW_QSTR(MP_QSTR_);
    {
        const char *p = path;
        for (mp_uint_t i = 1; i < path_num; i++) {
            const char *p1 = strchr(p, PATHLIST_SEP_CHAR);
            if (p1 == NULL) {
                p1 = p + strlen(p);
            }
            path_items[i] = mp_obj_new_str_via_qstr(p, p1 - p);
            p = p1 + 1;
        }
    }

    mp_obj_list_init(MP_OBJ_TO_PTR(mp_sys_argv), 0);
#endif

#if MICROPY_VFS_POSIX
	{
		// Mount the host FS at the root of our internal VFS
		mp_obj_t args[2] = {
			MP_OBJ_TYPE_GET_SLOT(&mp_type_vfs_posix, make_new)(&mp_type_vfs_posix, 0, 0, NULL),
			MP_OBJ_NEW_QSTR(MP_QSTR__slash_),
		};
		mp_vfs_mount(2, args, (mp_map_t *)&mp_const_empty_map);
		MP_STATE_VM(vfs_cur) = MP_STATE_VM(vfs_mount_table);
	}
#endif

    readline_init0();

    // initialise peripherals

#if MICROPY_PY_MACHINE_PIN    
    machine_pins_init();
#endif
#if MICROPY_PY_MACHINE_I2S
    machine_i2s_init0();
#endif

    // run boot-up scripts
	pyexec_frozen_module("_boot.py",false);
    pyexec_file_if_exists("boot.py");

    if (pyexec_mode_kind == PYEXEC_MODE_FRIENDLY_REPL) {
        int ret = pyexec_file_if_exists("main.py");
        if (ret & PYEXEC_FORCED_EXIT) {
            goto soft_reset_exit;
        }
    }

#if MICROPY_HELPER_REPL
    for (;;) {
        if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
            vprintf_like_t vprintf_log = esp_log_set_vprintf(vprintf_null);
            if (pyexec_raw_repl() != 0) {
                break;
            }
            esp_log_set_vprintf(vprintf_log);
        } else {
            if (pyexec_friendly_repl() != 0) {
                break;
           }
        }
    }
#endif

soft_reset_exit:

#if MICROPY_PY_MACHINE_TIMER    
    machine_timer_deinit_all();
#endif

#if MICROPY_PY_THREAD
    mp_thread_deinit();
#endif

#if MICROPY_EMIT_NATIVE || MICROPY_PERSISTENT_CODE_LOAD
	esp_native_code_free_all();
#endif

    gc_sweep_all();

    mp_hal_stdout_tx_str("MPY: soft reboot\r\n");

    // deinitialise peripherals
#if MICROPY_MACHINE_PWM
    machine_pwm_deinit_all();
#endif
#if MICROPY_PY_MACHINE_RMT
    // TODO: machine_rmt_deinit_all();
#endif
#if MICROPY_PY_MACHINE_PINS
    machine_pins_deinit();
#endif
#if MICROPY_PY_MACHINE
    machine_deinit();
#endif
//    usocket_events_deinit();

    mp_deinit();

	if (MP_Tasks.console[0]) {
		if (MP_Tasks.console_out) {
			fflush(MP_Tasks.console_out);
			fclose(MP_Tasks.console_out);
			MP_Tasks.console_out = NULL;
		}
		else
			fflush(stdout);

		if (MP_Tasks.console_in) {
			fclose(MP_Tasks.console_in);
			MP_Tasks.console_in = NULL;
		}

		MP_Tasks.in_fd = 0;
		MP_Tasks.out_fd = 1;
	} else
		fflush(stdout);

	goto soft_reset;
}

void mp_start(const char * Console) {
	if (Console && Console[0])
		strncpy(MP_Tasks.console, Console, sizeof(MP_Tasks.console));
	else
		MP_Tasks.console[0] = '\0';

    // Create and transfer control to the MicroPython task.
    MP_Tasks.handle = xTaskCreateStaticPinnedToCore(mp_task, "mp_task", MP_TASK_STACK_SIZE, NULL, MP_TASK_PRIORITY, MP_Tasks.stack, &MP_Tasks.buffer, APP_CPU_NUM);
}


/********************** mphalport.c *************************/
void nlr_jump_fail(void *val) {
    printf("NLR jump failed, val=%p\n", val);
    esp_restart();
}

#if MICROPY_EMIT_NATIVE || MICROPY_PERSISTENT_CODE_LOAD
static void esp_native_code_free_all(void) {
    while (native_code_head != NULL) {
        native_code_node_t *next = native_code_head->next;
        heap_caps_free(native_code_head);
        native_code_head = next;
    }
}

void *esp_native_code_commit(void *buf, size_t len, void *reloc) {
    len = (len + 3) & ~3;
    size_t len_node = sizeof(native_code_node_t) + len;
    native_code_node_t *node = heap_caps_malloc(len_node, MALLOC_CAP_EXEC);
    #if CONFIG_IDF_TARGET_ESP32S2
    // Workaround for ESP-IDF bug https://github.com/espressif/esp-idf/issues/14835
    if (node != NULL && !esp_ptr_executable(node)) {
        free(node);
        node = NULL;
    }
    #endif // CONFIG_IDF_TARGET_ESP32S2
    if (node == NULL) {
        m_malloc_fail(len_node);
    }
    node->next = native_code_head;
    native_code_head = node;
    void *p = node->data;
    if (reloc) {
        mp_native_relocate(reloc, buf, (uintptr_t)p);
    }
    memcpy(p, buf, len);
    return p;
}
#endif

// Receive single character, blocking until one is available.
int mp_hal_stdin_rx_chr(void) {
	unsigned char c = 0;
	int ret;

	if ((ret = read(MP_Tasks.in_fd, &c, 1) == 1)) {
		return c;
	}
	return 0;
}

// Send the string of given length.
mp_uint_t mp_hal_stdout_tx_strn(const char *str, size_t len) {
	return write(MP_Tasks.out_fd, str, len);
}

uintptr_t mp_hal_stdio_poll(uintptr_t poll_flags) {
	uintptr_t ret = 0;
	struct pollfd fds[3];
	int nfds=0;

	if (poll_flags & MP_STREAM_POLL_RD) {
		fds[nfds].fd = 0;
		fds[nfds].events = POLLIN;
		fds[nfds].revents = 0;
		nfds++;
	}

	if (poll_flags & MP_STREAM_POLL_WR) {
		fds[nfds].fd = 1;
		fds[nfds].events = POLLOUT;
		fds[nfds].revents = 0;
		nfds++;
	}

	if (!nfds)
		return 0;

	if ((ret = poll(fds,nfds,-1)) > 0) {
		printf("poll return %d (%d)\n",ret,errno);
		ret = 0;
		nfds = 0;

		if (poll_flags & MP_STREAM_POLL_RD) {
			if (fds[nfds].revents & POLLIN) {
				ret |= MP_STREAM_POLL_RD;
				printf("poll in event\n");
			}
			nfds++;
		}

		if (poll_flags & MP_STREAM_POLL_WR) {
			if (fds[nfds].revents & POLLOUT) {
				ret |= MP_STREAM_POLL_WR;
				printf("poll out event\n");
			}
			nfds++;
		}
	} else
		return 0;

	return ret;
}

/******************* mp_hal *************************/
uint32_t mp_hal_ticks_ms(void) {
    return esp_timer_get_time() / 1000;
}

uint32_t mp_hal_ticks_us(void) {
    return esp_timer_get_time();
}

void mp_hal_delay_ms(uint32_t ms) {
    uint64_t us = (uint64_t)ms * 1000ULL;
    uint64_t dt;
    uint64_t t0 = esp_timer_get_time();
    for (;;) {
        mp_handle_pending(true);
        MICROPY_PY_SOCKET_EVENTS_HANDLER
        MP_THREAD_GIL_EXIT();
        uint64_t t1 = esp_timer_get_time();
        dt = t1 - t0;
        if (dt + portTICK_PERIOD_MS * 1000ULL >= us) {
            // doing a vTaskDelay would take us beyond requested delay time
            taskYIELD();
            MP_THREAD_GIL_ENTER();
            t1 = esp_timer_get_time();
            dt = t1 - t0;
            break;
        } else {
            ulTaskNotifyTake(pdFALSE, 1);
            MP_THREAD_GIL_ENTER();
        }
    }
    if (dt < us) {
        // do the remaining delay accurately
        mp_hal_delay_us(us - dt);
    }
}

void mp_hal_delay_us(uint32_t us) {
    // these constants are tested for a 240MHz clock
    const uint32_t this_overhead = 5;
    const uint32_t pend_overhead = 150;

    // return if requested delay is less than calling overhead
    if (us < this_overhead) {
        return;
    }
    us -= this_overhead;

    uint64_t t0 = esp_timer_get_time();
    for (;;) {
        uint64_t dt = esp_timer_get_time() - t0;
        if (dt >= us) {
            return;
        }
        if (dt + pend_overhead < us) {
            // we have enough time to service pending events
            // (don't use MICROPY_EVENT_POLL_HOOK because it also yields)
            mp_handle_pending(true);
        }
    }
}

uint64_t mp_hal_time_ns(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t ns = tv.tv_sec * 1000000000ULL;
    ns += (uint64_t)tv.tv_usec * 1000ULL;
    return ns;
}

// Wake up the main task if it is sleeping.
void mp_hal_wake_main_task(void) {
    xTaskNotifyGive(mp_main_task_handle);
}

// Wake up the main task if it is sleeping, to be called from an ISR.
void mp_hal_wake_main_task_from_isr(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(mp_main_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

#if !MICROPY_VFS
mp_import_stat_t mp_import_stat(const char * filename) {
	struct stat stat_buff;
	int ret = stat(filename, &stat_buff);

	if (!ret) {
		if (stat_buff.st_mode & S_IFREG)
			return MP_IMPORT_STAT_FILE;
		if (stat_buff.st_mode & S_IFDIR)
			return MP_IMPORT_STAT_DIR;
	}

	return MP_IMPORT_STAT_NO_EXIST;
}

mp_obj_t mp_builtin_open(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
	return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(mp_builtin_open_obj, 1, mp_builtin_open);

#endif
