/*
 * SPDX-License-Identifier: MIT
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Marc CAPDEVILLE
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

// Options to control how MicroPython is built for this port,
// overriding defaults in py/mpconfig.h.

#include <stdint.h>
#include <alloca.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

#include "mpconfigboard.h"

// object representation and NLR handling
#define MICROPY_OBJ_REPR                    (MICROPY_OBJ_REPR_A)
#define MICROPY_NLR_SETJMP                  (1)
#if CONFIG_IDF_TARGET_ESP32C3
#define MICROPY_GCREGS_SETJMP               (1)
#endif

// memory allocation policies
#define MICROPY_ALLOC_PATH_MAX              (32)

// emitters
#if CONFIG_MICROPY_SYS_PATH_ARGV_DEFAULTS
#define MICROPY_PY_SYS_PATH_ARGV_DEFAULTS (1)
#else
#define MICROPY_PY_SYS_PATH_ARGV_DEFAULTS (0)
#define MICROPY_PY_SYS_PATH_DEFAULT CONFIG_MICROPY_SYS_PATH_DEFAULT
#endif

// workaround for xtensa-esp32-elf-gcc esp-2020r3, which can generate wrong code for loops
// see https://github.com/espressif/esp-idf/issues/9130
// this was fixed in newer versions of the compiler by:
//   "gas: use literals/const16 for xtensa loop relaxation"
//   https://github.com/jcmvbkbc/binutils-gdb-xtensa/commit/403b0b61f6d4358aee8493cb1d11814e368942c9
#define MICROPY_COMP_CONST_FOLDING_COMPILER_WORKAROUND (1)

// optimisations
#define MICROPY_OPT_COMPUTED_GOTO           (1)

// Python internal features
#define MICROPY_ENABLE_GC                   (1)
#define MICROPY_ENABLE_EMERGENCY_EXCEPTION_BUF (1)
#define MICROPY_LONGINT_IMPL                (MICROPY_LONGINT_IMPL_MPZ)
#define MICROPY_ERROR_REPORTING             (MICROPY_ERROR_REPORTING_NORMAL)
#define MICROPY_WARNINGS                    (1)
#define MICROPY_FLOAT_IMPL                  (MICROPY_FLOAT_IMPL_FLOAT)
#define MICROPY_STREAMS_POSIX_API           (1)
#define MICROPY_QSTR_EXTRA_POOL             mp_qstr_frozen_const_pool
#define MICROPY_USE_INTERNAL_ERRNO          (0) // errno.h from xtensa-esp32-elf/sys-include/sys
#define MICROPY_USE_INTERNAL_PRINTF         (0) // ESP32 SDK requires its own printf
#define MICROPY_SCHEDULER_DEPTH             (8)

// control over Python builtins
#define MICROPY_PY_STR_BYTES_CMP_WARN       (1)
#define MICROPY_PY_ALL_INPLACE_SPECIAL_METHODS (1)
#define MICROPY_PY_BUILTINS_HELP_TEXT       esp32_help_text
#define MICROPY_PY_IO_BUFFEREDWRITER        (1)
#define MICROPY_PY_UTIME_MP_HAL             (1)

#if CONFIG_MICROPY_ROM_LEVEL_MINIMUM
#define MICROPY_CONFIG_ROM_LEVEL	    (MICROPY_CONFIG_ROM_LEVEL_MINIMUM)
#endif

// Kconfig controled features
#if CONFIG_MICROPY_ROM_LEVEL_CORE
#define MICROPY_CONFIG_ROM_LEVEL	    (MICROPY_CONFIG_ROM_LEVEL_CORE_FEATURES)
#endif

#if CONFIG_MICROPY_ROM_LEVEL_BASIC
#define MICROPY_CONFIG_ROM_LEVEL	    (MICROPY_CONFIG_ROM_LEVEL_BASIC_FEATURES)
#endif

#if CONFIG_MICROPY_ROM_LEVEL_EXTRA
#define MICROPY_CONFIG_ROM_LEVEL	    (MICROPY_CONFIG_ROM_LEVEL_EXTRA_FEATURES)
#endif

#if CONFIG_MICROPY_ROM_LEVEL_FULL
#define MICROPY_CONFIG_ROM_LEVEL	    (MICROPY_CONFIG_ROM_LEVEL_FULL_FEATURES)
#endif

#if CONFIG_MICROPY_ROM_LEVEL_EVERYTHINGS
#define MICROPY_CONFIG_ROM_LEVEL	    (MICROPY_CONFIG_ROM_LEVEL_EVERYTHINGS)
#endif

#ifdef CONFIG_MICROPY_HW_BOARD_NAME
#define MICROPY_HW_BOARD_NAME CONFIG_MICROPY_HW_BOARD_NAME
#else
#define MICROPY_HW_BOARD_NAME "<default board name>"
#endif

#ifdef CONFIG_MICROPY_HW_MCU_NAME
#define MICROPY_HW_MCU_NAME CONFIG_MICROPY_HW_MCU_NAME
#else
#define MICROPY_HW_MCU_NAME CONFIG_IDF_TARGET
#endif

#if CONFIG_MICROPY_PERSISTENT_CODE_LOAD
#define MICROPY_PERSISTENT_CODE_LOAD (1)
#define MICROPY_MAKE_POINTER_CALLABLE(p) ((void *)((mp_uint_t)(p)))
#endif

#if CONFIG_MICROPY_EMIT_NATIVE_CODE && !CONFIG_IDF_TARGET_ESP32C3
#define MICROPY_EMIT_XTENSAWIN              (1)
#else
#define MICROPY_EMIT_XTENSAWIN (0)
#endif

#if MICROPY_PERISTENT_CODE_LOAD
#warn "PERSISTENT CODE LOAD"
#endif

#if MICROPY_EMIT_XTENSAWIN || MICROPY_PERSISTENT_CODE_LOAD
void *esp_native_code_commit(void *, size_t, void *);
#define MP_PLAT_COMMIT_EXEC(buf, len, reloc) esp_native_code_commit(buf, len, reloc)
#endif


#if CONFIG_MICROPY_THREAD
#define MICROPY_PY_THREAD                   (1)
#define MICROPY_PY_THREAD_GIL               (1)
#define MICROPY_PY_THREAD_GIL_VM_DIVISOR    (32)
#endif

#if CONFIG_MICROPY_BUILTINS_BYTEARRAY
#define MICROPY_PY_BUILTINS_BYTEARRAY	(1)
#endif

#if CONFIG_MICROPY_EXTERNAL_MODULE
#define MICROPY_ENABLE_EXTERNAL_IMPORT        (1)
#endif

#if CONFIG_MICROPY_MODULE_FROZEN
#define MICROPY_MODULE_FROZEN         (1)
#define MICROPY_MODULE_FROZEN_MPY     (1)
#endif

#if CONFIG_MICROPY_WEAK_LINKS
#define MICROPY_MOCULE_WEAK_LINKS (1)
#endif

#if CONFIG_MICROPY_BUILTIN_INIT
#define MICROPY_MOCULE_BUILTIN_INIT (1)
#endif

#if CONFIG_MICROPY_IO
#define MICROPY_PY_IO	(1)
#endif

#if CONFIG_MICROPY_VFS
#define MICROPY_VFS		(1)
#define MICROPY_READER_VFS                  (1)

#if CONFIG_MICROPY_PY_VFS
#define MICROPY_PY_VFS (1)
#endif

#if CONFIG_MICROPY_VFS_POSIX
#define MICROPY_VFS_POSIX	(1)
#define MICROPY_READER_POSIX        (1)
#define MP_HAL_RETRY_SYSCALL(ret, syscall, raise) { \
        for (;;) { \
            MP_THREAD_GIL_EXIT(); \
            ret = syscall; \
            MP_THREAD_GIL_ENTER(); \
            if (ret == -1) { \
                int err = errno; \
                if (err == EINTR) { \
                    mp_handle_pending(true); \
                    continue; \
                } \
                raise; \
            } \
            break; \
        } \
}

#endif

#if CONFIG_MICROPY_FINALISER || CONFIG_MICROPY_VFS_POSIX
#define MICROPY_ENABLE_FINALISER	(1)
#endif

#if CONFIG_MICROPY_VFS_VFAT
#define MICROPY_VFS_FAT	(1)
#define MICROPY_FATFS_ENABLE_LFN            (1)
#define MICROPY_FATFS_RPATH                 (2)
#define MICROPY_FATFS_MAX_SS                (4096)
#define MICROPY_FATFS_LFN_CODE_PAGE         437 /* 1=SFN/ANSI 437=LFN/U.S.(OEM) */
#endif

#if CONFIG_MICROPY_VFS_LFS1
#define MICROPY_VFS_LFS1	(1)
#define LFS1_NO_MALLOC
#define LFS1_NO_DEBUG
#define LFS1_NO_WARN
#define LFS1_NO_ERROR
#define LFS1_NO_ASSERT
#endif

#if CONFIG_MICROPY_VFS_LFS2
#define MICROPY_VFS_LFS2	(1)
#define LFS2_NO_MALLOC
#define LFS2_NO_DEBUG
#define LFS2_NO_WARN
#define LFS2_NO_ERROR
#define LFS2_NO_ASSERT
#endif

#if CONFIG_MICROPY_VFS_POSIX
#define mp_type_fileio mp_type_vfs_posix_fileio
#define mp_type_textio mp_type_vfs_posix_textio
#elif MICROPY_VFS_FAT
#define mp_type_fileio mp_type_vfs_fat_fileio
#define mp_type_textio mp_type_vfs_fat_textio
#elif MICROPY_VFS_LFS1
#define mp_type_fileio mp_type_vfs_lfs1_fileio
#define mp_type_textio mp_type_vfs_lfs1_textio
#elif MICROPY_VFS_LFS2
#define mp_type_fileio mp_type_vfs_lfs2_fileio
#define mp_type_textio mp_type_vfs_lfs2_textio
#endif
#else
#endif

#if CONFIG_MICROPY_HELPER_REPL
#define MICROPY_HELPER_REPL	(1)
#if CONFIG_MICROPY_REPL_EMACS_KEYS
#define MICROPY_REPL_EMACS_KEYS (1)
#endif
#if CONFIG_MICROPY_REPL_AUTO_INDENT
#define MICROPY_REPL_AUTO_INDENT (1)
#endif
#endif

#if CONFIG_MICROPY_COMPILER
#define MICROPY_ENABLE_COMPILER	(1)
#if CONFIG_MICROPY_VFS
#define MICROPY_PY_BUILTINS_EXECFILE (1)
#endif
#endif

#if CONFIG_MICROPY_SCHEDULER
#define MICROPY_ENABLE_SCHEDULER (1)
#endif

#if CONFIG_MICROPY_MACHINE
#define MICROPY_PY_MACHINE (1)
#endif

#if CONFIG_MICROPY_MACHINE_RTC
#define MICROPY_MACHINE_RTC	(1)
#endif

#if CONFIG_MICROPY_MACHINE_SIGNAL
#define MICROPY_MACHINE_SIGNAL	(1)
#endif

#if CONFIG_MICROPY_MACHINE_PIN
#define MICROPY_MACHINE_PIN	(1)
#define MICROPY_PY_MACHINE_PIN_MAKE_NEW     mp_pin_make_new
#endif

#if CONFIG_MICROPY_MACHINE_PULSE
#define MICROPY_PY_MACHINE_PULSE	(1)
#endif

#if CONFIG_MICROPY_MACHINE_TIMER
#define MICROPY_PY_MACHINE_TIMER	(1)
#endif

#if CONFIG_MICROPY_GC
#define MICROPY_PY_GC	(1)
#define MICROPY_ENABLE_EXTERNAL_IMPORT (1)
#endif

#if CONFIG_MICROPY_SYS
#define MICROPY_PY_SYS (1)
#endif

#if CONFIG_MICROPY_UPLATFORM
#define MICROPY_PY_UPLATFORM (1)
#endif

#if CONFIG_MICROPY_BTREE
#ifndef MICROPY_PY_BTREE
#define MICROPY_PY_BTREE (1)
#endif
#endif

#if CONFIG_MICROPY_OS
#define MICROPY_PY_OS	(1)
#define MICROPY_PY_OS_INCLUDEFILE "ports/esp32/modos.c"	
#define MICROPY_ENABLE_EXTERNAL_IMPORT (1)
#define MICROPY_PY_OS_DUPTERM               (0)
#define MICROPY_PY_OS_DUPTERM_NOTIFY       (0)
#define MICROPY_PY_OS_UNAME                (1)
#define MICROPY_PY_OS_URANDOM              (1)
#include "esp_random.h"
#define MICROPY_PY_URANDOM_SEED_INIT_FUNC   (esp_random())
#define MICROPY_PY_ATTRTUPLE (1)
#define MICROPY_KBD_EXCEPTION (1)
#define MICROPY_PY_OS_STATVFS (0)
#endif

#if CONFIG_MICROPY_UTIME
#define MICROPY_PY_UTIME_MP_HAL             (1)
#define MICROPY_PY_UTIMEQ                   (1)
#endif

#if CONFIG_MICROPY_MACHINE_BITSTREAM	
#define MICROPY_PY_MACHINE_BITSTREAM        (1)
#endif

#if CONFIG_MICROPY_MACHINE_PWM 
#define MICROPY_PY_MACHINE_PWM              (1)
#define MICROPY_PY_MACHINE_PWM_INIT         (1)
#define MICROPY_PY_MACHINE_PWM_DUTY         (0)
#define MICROPY_PY_MACHINE_PWM_DUTY_U16_NS  (0)
#define MICROPY_PY_MACHINE_PWM_INCLUDEFILE  "ports/esp32/machine_pwm.c"
#endif

#if CONFIG_MICROPY_MACHINE_I2C
#define MICROPY_PY_MACHINE_I2C              (1)
#define MICROPY_PY_MACHINE_I2C_TRANSFER_WRITE1 (1)
#endif

#if CONFIG_MICROPY_MACHINE_SOFTI2C
#define MICROPY_PY_MACHINE_SOFTI2C          (1)
#endif

#if CONFIG_MICROPY_MACHINE_SPI
#define MICROPY_PY_MACHINE_SPI              (1)
#define MICROPY_PY_MACHINE_SPI_MSB          (1)
#define MICROPY_PY_MACHINE_SPI_LSB          (0)
#endif

#if CONFIG_MICROPY_MACHINE_SOFTSPI
#define MICROPY_PY_MACHINE_SOFTSPI              (1)
#define MICROPY_HW_SOFTSPI_MIN_DELAY        (0)
#define MICROPY_HW_SOFTSPI_MAX_BAUDRATE     (ets_get_cpu_frequency() * 1000000 / 200) // roughly
#endif

#if CONFIG_MICROPY_MACHINE_DAC
#define MICROPY_PY_MACHINE_DAC              (1)
#endif

#if CONFIG_MICROPY_MACHINE_I2S
#define MICROPY_PY_MACHINE_I2S              (1)
#endif

#if 0
// extended modules
#define MICROPY_PY_UHASHLIB_SHA1            (0)
#define MICROPY_PY_UHASHLIB_SHA256          (0)
#define MICROPY_PY_UCRYPTOLIB               (0)


#ifndef MICROPY_PY_NETWORK_WLAN
#define MICROPY_PY_NETWORK_WLAN             (0)
#endif

#ifndef MICROPY_HW_ENABLE_SDCARD
#define MICROPY_HW_ENABLE_SDCARD            (0)
#endif

#define MICROPY_PY_USSL                     (0)
#define MICROPY_SSL_MBEDTLS                 (0)
#define MICROPY_PY_USSL_FINALISER           (0)

#define MICROPY_PY_UWEBSOCKET               (0)
#define MICROPY_PY_WEBREPL                  (0)

#define MICROPY_PY_BTREE                    (1)

#define MICROPY_PY_ONEWIRE                  (0)

#define MICROPY_PY_UPLATFORM                (1)

#define MICROPY_PY_SOCKET_EVENTS           (MICROPY_PY_WEBREPL)

#ifndef MICROPY_PY_BLUETOOTH
#define MICROPY_PY_BLUETOOTH                (0)
#define MICROPY_PY_BLUETOOTH_ENABLE_CENTRAL_MODE (1)
#define MICROPY_BLUETOOTH_NIMBLE            (1)
#define MICROPY_BLUETOOTH_NIMBLE_BINDINGS_ONLY (1)
#define MICROPY_PY_BLUETOOTH_RANDOM_ADDR    (1)
#define MICROPY_PY_BLUETOOTH_DEFAULT_GAP_NAME ("ESP32")
#endif

#endif

#define MP_STATE_PORT MP_STATE_VM

#if CONFIG_MICROPY_MACHINE_PIN
#define MICROPY_PORT_ROOT_POINTER_MACHINE_PIN_IRQ mp_obj_t machine_pin_irq_handler[40];
#else
#define MICROPY_PORT_ROOT_POINTER_MACHINE_PIN_IRQ
#endif

#if CONFIG_MICROPY_MACHINE_TIMER
struct _machine_timer_obj_t;
#define MICROPY_PY_MACHINE_TIMER (1)
#define MICROPY_PORT_ROOT_POINTER_MACHINE_TIMER struct _machine_timer_obj_t *machine_timer_obj_head;
#else
#define MICROPY_PORT_ROOT_POINTER_MACHINE_TIMER
#endif

#if CONFIG_MICROPY_MACHINE_I2S
#include "driver/i2s.h"
#define MICROPY_PORT_ROOT_POINTER_MACHINE_I2S struct _machine_i2s_obj_t *machine_i2s_obj[SOC_I2S_NUM];
#else
#define MICROPY_PORT_ROOT_POINTER_MACHINE_I2S
#endif

#if MICROPY_BLUETOOTH_NIMBLE
struct mp_bluetooth_nimble_root_pointers_t;
#define MICROPY_PORT_ROOT_POINTER_BLUETOOTH_NIMBLE struct _mp_bluetooth_nimble_root_pointers_t *bluetooth_nimble_root_pointers;
#else
#define MICROPY_PORT_ROOT_POINTER_BLUETOOTH_NIMBLE
#endif

#if MICROPY_EMIT_XTENSAWIN || MICROPY_PERSISTENT_CODE_LOAD
#define MICROPY_PORT_ROOT_POINTER_NATIVE_CODE mp_obj_t native_code_pointers;
#else
#define MICROPY_PORT_ROOT_POINTER_NATIVE_CODE
#endif

#ifndef MICROPY_USER_ROOT_POINTERS
#define MICROPY_USER_ROOT_POINTERS
#endif

#define MICROPY_PORT_ROOT_POINTERS \
    const char *readline_hist[8]; \
    MICROPY_PORT_ROOT_POINTER_NATIVE_CODE \
    MICROPY_PORT_ROOT_POINTER_MACHINE_PIN_IRQ \
    MICROPY_PORT_ROOT_POINTER_MACHINE_TIMER \
    MICROPY_PORT_ROOT_POINTER_MACHINE_I2S \
    MICROPY_PORT_ROOT_POINTER_BLUETOOTH_NIMBLE \
    MICROPY_USER_ROOT_POINTERS

// type definitions for the specific machine

#define MP_SSIZE_MAX (0x7fffffff)

#if MICROPY_PY_SOCKET_EVENTS
#define MICROPY_PY_SOCKET_EVENTS_HANDLER extern void socket_events_handler(void); socket_events_handler();
#else
#define MICROPY_PY_SOCKET_EVENTS_HANDLER
#endif

#if MICROPY_PY_THREAD
#define MICROPY_EVENT_POLL_HOOK \
    do { \
        extern void mp_handle_pending(bool); \
        mp_handle_pending(true); \
        MICROPY_PY_SOCKET_EVENTS_HANDLER \
        MP_THREAD_GIL_EXIT(); \
        ulTaskNotifyTake(pdFALSE, 1); \
        MP_THREAD_GIL_ENTER(); \
    } while (0);
#else
#define MICROPY_EVENT_POLL_HOOK \
    do { \
        extern void mp_handle_pending(bool); \
        mp_handle_pending(true); \
        MICROPY_PY_SOCKET_EVENTS_HANDLER \
        asm ("waiti 0"); \
    } while (0);
#endif

// Functions that should go in IRAM
#define MICROPY_WRAP_MP_BINARY_OP(f) IRAM_ATTR f
#define MICROPY_WRAP_MP_EXECUTE_BYTECODE(f) IRAM_ATTR f
#define MICROPY_WRAP_MP_LOAD_GLOBAL(f) IRAM_ATTR f
#define MICROPY_WRAP_MP_LOAD_NAME(f) IRAM_ATTR f
#define MICROPY_WRAP_MP_MAP_LOOKUP(f) IRAM_ATTR f
#define MICROPY_WRAP_MP_OBJ_GET_TYPE(f) IRAM_ATTR f
#define MICROPY_WRAP_MP_SCHED_EXCEPTION(f) IRAM_ATTR f
#define MICROPY_WRAP_MP_SCHED_KEYBOARD_INTERRUPT(f) IRAM_ATTR f

#define UINT_FMT "%u"
#define INT_FMT "%d"

typedef int32_t mp_int_t; // must be pointer size
typedef uint32_t mp_uint_t; // must be pointer size
typedef long mp_off_t;
// ssize_t, off_t as required by POSIX-signatured functions in stream.h
#include <sys/types.h>

// board specifics
#define MICROPY_PY_SYS_PLATFORM "esp32"

// ESP32-S3 extended IO for 47 & 48
#ifndef MICROPY_HW_ESP32S3_EXTENDED_IO
#define MICROPY_HW_ESP32S3_EXTENDED_IO      (1)
#endif

#ifndef MICROPY_HW_ENABLE_MDNS_QUERIES
#define MICROPY_HW_ENABLE_MDNS_QUERIES      (0)
#endif

#ifndef MICROPY_HW_ENABLE_MDNS_RESPONDER
#define MICROPY_HW_ENABLE_MDNS_RESPONDER    (1)
#endif
