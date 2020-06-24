/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

/**
 * @file dw1000_dev.h
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief Device file
 *
 * @details This is the dev base class which utilises the functions to perform initialization and necessary configurations on device.
 *
 */

/** @ingroup api_dw1000
 *  This is the dev base class which utilises the functions to perform initialization and necessary configurations on device.
 *  @{
 */


#ifndef _DW1000_DEV_H_
#define _DW1000_DEV_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <uwb/uwb.h>
#include <os/os_dev.h>
#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_stats.h>
#include <dpl/dpl.h>

#define DWT_DEVICE_ID   (0xDECA0130) //!< Decawave Device ID
#define DWT_SUCCESS (0)              //!< DWT Success
#define DWT_ERROR   (-1)             //!< DWT Error
#define DWT_TIME_UNITS          (1.0/499.2e6/128.0) //!< DWT time units calculation

//! Device control status bits.
typedef struct _dw1000_dev_control_t{
    uint32_t wait4resp_enabled:1;           //!< Wait for the response
    uint32_t wait4resp_delay_enabled:1;     //!< Wait for the delayed response
    uint32_t delay_start_enabled:1;         //!< Transmit after delayed start
    uint32_t autoack_delay_enabled:1;       //!< Enables automatic acknowledgement feature with delay
    uint32_t start_rx_syncbuf_enabled:1;    //!< Enables receive syncbuffer
    uint32_t rx_timeout_enabled:1;          //!< Enables receive timeout
    uint32_t on_error_continue_enabled:1;   //!< Enables on_error_continue
    uint32_t sleep_after_tx:1;              //!< Enables to load LDE microcode on wake up
    uint32_t sleep_after_rx:1;              //!< Enables to load LDO tune value on wake up
    uint32_t cir_enable:1;                  //!< Enables reading CIR on this operation
    uint32_t rxauto_disable:1;              //!< Disable auto receive parameter
    uint32_t abs_timeout:1;                 //!< RX absolute timeout active
}dw1000_dev_control_t;


//! DW1000 receiver diagnostics parameters.
typedef struct _dw1000_dev_rxdiag_t{
    struct uwb_dev_rxdiag rxd;
    union {
        struct _rx_time {
            uint32_t    fp_idx:16;          //!< First path index (10.6 bits fixed point integer)
            uint32_t    fp_amp:16;          //!< Amplitude at floor(index FP) + 1
        };
        uint32_t rx_time;
    };
    union {
        struct _rx_fqual {
            uint64_t    rx_std:16;          //!<  Standard deviation of noise
            uint64_t    fp_amp2:16;         //!<  Amplitude at floor(index FP) + 2
            uint64_t    fp_amp3:16;         //!<  Amplitude at floor(index FP) + 3
            uint64_t    cir_pwr:16;         //!<  Channel Impulse Response max growth CIR
        };
        uint64_t rx_fqual;
    };
    uint16_t    pacc_cnt;                   //!<  Count of preamble symbols accumulated
} __attribute__((packed, aligned(1))) dw1000_dev_rxdiag_t;

#if MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN)
#define DW1000_SYS_STATUS_BT_PTR(_I) _I->sys_status_bt[_I->sys_status_bt_idx%MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN)]
#define DW1000_SYS_STATUS_BT_ADD(_I,_S,_U) _I->sys_status_bt[++_I->sys_status_bt_idx%MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN)] = \
        (struct dw1000_sys_status_backtrace) {.utime=_U, .sys_status_lo=_S}
#define DW1000_SYS_STATUS_BT_HI(_I,_S) DW1000_SYS_STATUS_BT_PTR(_I).sys_status_hi = _S
#define DW1000_SYS_STATUS_BT_FCTRL(_I,_FCTRL) DW1000_SYS_STATUS_BT_PTR(_I).fctrl = _FCTRL
#if MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_HI)
#define DW1000_SYS_STATUS_ASSEMBLE(_P) (((uint64_t)_P->sys_status_hi)<<32)|(uint64_t)_P->sys_status_lo
#define DW1000_SYS_STATUS_ASSEMBLE_LEN (5)
#else
#define DW1000_SYS_STATUS_ASSEMBLE(_P) (uint64_t)_P->sys_status_lo
#define DW1000_SYS_STATUS_ASSEMBLE_LEN (4)
#endif

//! DW1000 Interrupt backtrace structure
struct dw1000_sys_status_backtrace {
    uint32_t utime;
    uint32_t sys_status_lo;
    uint8_t sys_status_hi;
    uint8_t interrupt_reentry:1;
    uint16_t fctrl;
    uint32_t utime_end;
};
#endif

#if MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN)
#define DW1000_SPI_BT_PTR(_I) _I->spi_bt[_I->spi_bt_idx%MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN)]
#define DW1000_SPI_BT_ADD(_I,_CMD,_CLEN,_DATA,_DLEN,_IS_WR,_NB) if(!_I->spi_bt_lock){struct dw1000_spi_backtrace *p=&_I->spi_bt[++_I->spi_bt_idx%MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN)]; \
        *p = (struct dw1000_spi_backtrace){.utime=dpl_cputime_get32(), .cmd_len=_CLEN, .data_len=_DLEN, .is_write=_IS_WR, .non_blocking=_NB}; \
        memcpy(p->cmd,_CMD,_CLEN);_I->spi_bt_datap=_DATA;}
#define DW1000_SPI_BT_ADD_END(_I) if(!_I->spi_bt_lock){struct dw1000_spi_backtrace *p=&DW1000_SPI_BT_PTR(_I); \
        p->utime_end=dpl_cputime_get32();                                 \
        memcpy(p->data,_I->spi_bt_datap,(p->data_len>sizeof(p->data))?sizeof(p->data):p->data_len);}

//! DW1000 SPI backtrace structure
struct dw1000_spi_backtrace {
    uint32_t utime;
    uint8_t cmd[4];
    uint8_t cmd_len;
    uint8_t is_write:1;
    uint8_t non_blocking:1;
    uint8_t data[MYNEWT_VAL(DW1000_SPI_BACKTRACE_DATA_LEN)];
    uint16_t data_len;
    uint32_t utime_end;
};
#else
#define DW1000_SPI_BT_PTR(_I) {}
#define DW1000_SPI_BT_ADD(_I,_CMD,_CLEN,_DATA,_DLEN,_IS_WR,_NB) {}
#define DW1000_SPI_BT_ADD_END(_I) {}
#endif



struct _dw1000_dev_instance_t;

//! Device instance parameters.
typedef struct _dw1000_dev_instance_t{
    struct uwb_dev uwb_dev;                     //!< Common generalising struct uwb_dev

    struct dpl_sem * spi_sem;                   //!< Pointer to global spi bus semaphore
    struct dpl_sem spi_nb_sem;                  //!< Semaphore for nonblocking rd/wr operations
    int spi_baudrate;                           //!< SPI Baudrate (<20MHz)
    int spi_baudrate_low;                       //!< Low SPI Baudrate (<2MHz)
    uint8_t spi_num;                            //!< SPI number
    uint8_t irq_pin;                            //!< Interrupt request pin
    uint8_t ss_pin;                             //!< Slave select pin
    uint8_t rst_pin;                            //!< Reset pin

    struct dpl_sem tx_sem;                      //!< semphore for low level mac/phy functions
    struct dpl_mutex mutex;                     //!< mutex
    uint32_t part_id;                           //!< Identifier of a particular part design
    uint32_t lot_id;                            //!< Identification number assigned to a particular quantity

    uint16_t otp_rev;              //!< OTP parameter revision
    uint8_t otp_vbat;              //!< OTP parameter for voltage
    uint8_t otp_temp;              //!< OTP parameter for temperature
    uint8_t otp_xtal_trim;         //!< OTP Crystal trim
    uint32_t sys_cfg_reg;          //!< System config register
    uint32_t tx_fctrl;             //!< Transmit frame control register parameter
    uint32_t sys_status;           //!< SYS_STATUS_ID for current event
    uint8_t  sys_status_hi;        //!< SYS_STATUS_ID+4 for current event

    struct hal_spi_settings spi_settings;  //!< Structure of SPI settings in hal layer
#if MYNEWT_VAL(CIR_ENABLED)
    struct cir_dw1000_instance * cir;           //!< CIR instance (duplicate of uwb_dev->cir)
#endif
    dw1000_dev_rxdiag_t rxdiag;                    //!< DW1000 receive diagnostics
    dw1000_dev_control_t control;                  //!< DW1000 device control parameters

#if MYNEWT_VAL(DW1000_LWIP)
    void (* lwip_rx_complete_cb) (struct _dw1000_dev_instance_t *);
#endif
#if MYNEWT_VAL(DW1000_MAC_STATS)
    STATS_SECT_DECL(mac_stat_section) stat;
#endif
#if MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN)
    struct dw1000_sys_status_backtrace sys_status_bt[MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN)];
    uint16_t sys_status_bt_idx;
    uint8_t sys_status_bt_lock;
#endif
#if MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN)
    struct dw1000_spi_backtrace spi_bt[MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN)];
    uint16_t spi_bt_idx;
    uint8_t spi_bt_lock;
    uint8_t* spi_bt_datap;
#endif
#if MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN) || MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN)
    /* To allow translation from ticks to usecs in gdb during backtrace*/
    uint32_t bt_ticks2usec;
#endif
} dw1000_dev_instance_t;

//! SPI and other init parameters
struct dw1000_dev_cfg {
    struct dpl_sem *spi_sem;      //!< Pointer to os_sem structure to lock spi bus

    int spi_baudrate;             //!< SPI Baudrate (<20MHz)
    int spi_baudrate_low;         //!< Low SPI Baudrate (<2MHz)
    uint8_t spi_num;              //!< SPI number
    uint8_t rst_pin;              //!< Reset pin
    uint8_t irq_pin;              //!< Interrupt request pin
    uint8_t ss_pin;               //!< Slave select pin

    uint16_t rx_antenna_delay;    //!< Receive antenna delay
    uint16_t tx_antenna_delay;    //!< Transmit antenna delay
    int32_t  ext_clock_delay;     //!< External clock delay
};

int dw1000_dev_init(struct os_dev *odev, void *arg);
void dw1000_dev_deinit(dw1000_dev_instance_t * inst);
int dw1000_dev_config(dw1000_dev_instance_t * inst);
void dw1000_softreset(dw1000_dev_instance_t * inst);
struct uwb_dev_status dw1000_read(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length);
struct uwb_dev_status dw1000_write(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length);
uint64_t dw1000_read_reg(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, size_t nsize);
void dw1000_write_reg(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint64_t val, size_t nsize);
void dw1000_dev_set_sleep_timer(dw1000_dev_instance_t * inst, uint16_t count);
void dw1000_dev_configure_sleep(dw1000_dev_instance_t * inst);
struct uwb_dev_status dw1000_dev_enter_sleep(dw1000_dev_instance_t * inst);
struct uwb_dev_status dw1000_dev_wakeup(dw1000_dev_instance_t * inst);
struct uwb_dev_status dw1000_dev_enter_sleep_after_tx(dw1000_dev_instance_t * inst, uint8_t enable);
struct uwb_dev_status dw1000_dev_enter_sleep_after_rx(dw1000_dev_instance_t * inst, uint8_t enable);

#define dw1000_dwt_usecs_to_usecs(_t) (double)( (_t) * (0x10000UL/(128*499.2)))
#define dw1000_usecs_to_dwt_usecs(_t) (double)( (_t) / dw1000_dwt_usecs_to_usecs(1.0))

/**
 * @}
 *
 */

#ifdef __cplusplus
}
#endif

#endif /* _DW1000_DEV_H_ */
