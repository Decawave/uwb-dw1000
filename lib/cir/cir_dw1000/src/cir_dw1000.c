/**
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
 * @file cir.c
 * @author UWB Core <uwbcore@gmail.com>
 * @date Nov 29 2019
 * @brief Channel Impulse Response
 *
 * @details
 *
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <dpl/dpl.h>
#include <dpl/dpl_cputime.h>
#if __KERNEL__
#include <dpl/dpl_eventq.h>
#endif
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <stats/stats.h>

#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_stats.h>
#include <cir/cir_json.h>
#include <cir_dw1000/cir_dw1000.h>
#include <cir_dw1000/cir_dw1000_encode.h>

#if MYNEWT_VAL(CIR_STATS)
STATS_NAME_START(cir_dw1000_stat_section)
    STATS_NAME(cir_dw1000_stat_section, complete)
STATS_NAME_END(cir_dw1000_stat_section)
#define CIR_STATS_INC(__X) STATS_INC(cir->stat, __X)
#else
#define CIR_STATS_INC(__X) {}
#endif


#if MYNEWT_VAL(CIR_VERBOSE)
static void
cir_complete_ev_cb(struct dpl_event *ev)
{
    uint16_t i;
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)dpl_event_get_arg(ev);
    cir_dw1000_t * cir  = &inst->cir->cir;

    struct cir_json json = {
        .type = "ipatov",
        .idx = 0,
        .utime = inst->uwb_dev.rxtimestamp,
        .raw_ts = inst->cir->raw_ts,
        .resampler_delay = inst->cir->resampler_delay,
        .accumulator_count = inst->cir->pacc_cnt,
        .angle  = DPL_FLOAT64_FROM_F32(inst->cir->angle),
        .fp_idx = DPL_FLOAT64_FROM_F32(inst->cir->fp_idx),
        .fp_power = DPL_FLOAT64_FROM_F32(inst->cir->fp_power),
        .cir_offset = inst->cir->offset,
        .cir_count = inst->cir->length,
    };

    for (i=0; i < json.cir_count; i++) {
        json.real[i] = cir->array[i].real;
        json.imag[i] = cir->array[i].imag;
    }
    cir_json_write(&json);

#ifdef __KERNEL__
    size_t n = strlen(json.iobuf);
    json.iobuf[n]='\n';
    json.iobuf[n+1]='\0';
    cir_chrdev_output(json.iobuf, n+1);
#else
    printf("%s\n", json.iobuf);
#endif
}

#endif //CIR_VERBOSE


/**
 * @fn cir_dw1000_remap_fp_index(struct cir_dw1000_instance *cir0, struct cir_dw1000_instance *cir1)
 *
 * @brief Map cir0's fp_idx into cir1 and return it
 *
 * input parameters
 * @param cir0 - The cir which fp_index will be remapped
 * @param cir1 - The mapping target cir
 *
 * output parameters
 *
 * returns dpl_float32_t cir0.fp_idx as it would have been inside cir1
 */
dpl_float32_t
cir_dw1000_remap_fp_index(struct cir_dw1000_instance *cir0, struct cir_dw1000_instance *cir1)
{
    dpl_float64_t raw_ts_diff, tmp, tmp64;
    dpl_float32_t fp_idx_0_given_1;
    /* Correct aligment using raw timestamp and resampler delays */
    tmp64 = DPL_FLOAT64_INIT(64.0);
    tmp = DPL_FLOAT64_I64_TO_F64((int64_t) cir0->raw_ts + ((int64_t)cir0->resampler_delay)*8 -
                                 ((int64_t)cir1->raw_ts + ((int64_t)cir1->resampler_delay)*8));
    raw_ts_diff = DPL_FLOAT64_DIV(tmp, tmp64);

    /* Compensate for different clock and rx-antenna delays */
    if (cir0->dev_inst && cir1->dev_inst) {
        raw_ts_diff = DPL_FLOAT64_ADD(raw_ts_diff,
                                       DPL_FLOAT64_DIV(
                                           DPL_FLOAT64_I32_TO_F64((int32_t)cir0->dev_inst->uwb_dev.ext_clock_delay  -
                                                                  (int32_t)cir1->dev_inst->uwb_dev.ext_clock_delay),
                                           tmp64)
            );
        raw_ts_diff = DPL_FLOAT64_ADD(raw_ts_diff,
                                       DPL_FLOAT64_DIV(
                                           DPL_FLOAT64_I32_TO_F64((int32_t)cir0->dev_inst->uwb_dev.rx_antenna_delay -
                                                                  (int32_t)cir1->dev_inst->uwb_dev.rx_antenna_delay),
                                           tmp64)
            );
    }

    fp_idx_0_given_1 = DPL_FLOAT32_ADD(cir0->fp_idx, DPL_FLOAT32_FROM_F64(raw_ts_diff));
    return fp_idx_0_given_1;
}

/**
 * @fn read_from_acc(struct cir_dw1000_instance * cir, uint16_t fp_idx)
 *
 * @brief Helper function to read CIR data from accumulator
 *
 * input parameters
 * @param cir - instance pointer
 * @param fp_idx - where to read
 *
 */
static void
read_from_acc(struct cir_dw1000_instance * cir, uint16_t fp_idx)
{
    /* Sanity check, only a fp_index within the accumulator makes sense */
    cir->offset = cir->dev_inst->uwb_dev.config.rx.cirOffset;
    cir->length = cir->dev_inst->uwb_dev.config.rx.cirSize;

    /* Really a duplicate as uwbcfg sets config.cir_enable to false if length is 0 */
    if (!cir->length) {
        /* Nothing to do */
        return;
    }

    /* If we can't extract CIR from required offset, start at zero */
    if(fp_idx < cir->offset) {
        cir->offset = fp_idx;
    }

    /* If we can't extract full length CIR, truncate */
    if((fp_idx - cir->offset + cir->length) > 1023) {
        cir->length = 1023 - (fp_idx - cir->offset);
    }

    if(cir->length > MYNEWT_VAL(CIR_MAX_SIZE)) {
        cir->length = MYNEWT_VAL(CIR_MAX_SIZE);
    }
    dw1000_read_accdata(cir->dev_inst, (uint8_t *)&cir->cir, (fp_idx - cir->offset) * sizeof(cir_dw1000_complex_t),
                        1 + cir->length * sizeof(struct  _cir_dw1000_complex_t));

#ifndef __KERNEL__
    cir->angle = atan2f((dpl_float32_t)cir->cir.array[cir->offset].imag,
                        (dpl_float32_t)cir->cir.array[cir->offset].real);
#else
    dpl_float64_t tmp = DPL_FLOAT64_ATAN(
        DPL_FLOAT64_DIV(DPL_FLOAT64_I32_TO_F64((int32_t)cir->cir.array[cir->offset].imag),
                        DPL_FLOAT64_I32_TO_F64((int32_t)cir->cir.array[cir->offset].real)));
    cir->angle = DPL_FLOAT32_FROM_F64(tmp);
#endif
    cir->cir_inst.status.valid = 1;
}

bool
cir_dw1000_reread_from_cir(dw1000_dev_instance_t * inst, struct cir_dw1000_instance *master_cir)
{
    /* CIR-data is lost already if the receiver has been turned back on */
    if (inst->uwb_dev.status.rx_restarted) {
        return false;
    }
    struct cir_dw1000_instance * cir = inst->cir;

    /* Correct aligment by remapping master_cir's fp_idx into our cir */
    dpl_float32_t fp_idx_override  = cir_dw1000_remap_fp_index(master_cir, cir);

    /* Sanity check, only a fp_index within the accumulator makes sense */
    if(DPL_FLOAT32_INT(fp_idx_override) < 0 ||
       DPL_FLOAT32_INT(fp_idx_override) > 1023) {
        /* Can't extract CIR from required offset, abort */
        cir->cir_inst.status.valid = 0;
        return false;
    }

    /* Override our local LDE result with the other LDE's result */
    cir->cir_inst.status.lde_override = 1;
    uint16_t fp_idx = DPL_FLOAT32_INT(DPL_FLOAT32_CEIL(fp_idx_override));

    read_from_acc(cir, fp_idx);
    return true;
}

/*!
 * @fn cir_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
 *
 * @brief Read CIR inadvance of RXENB
 *
 * input parameters
 * @param inst - struct uwb_dev * inst
 *
 * output parameters
 *
 * returns none
 */
#if MYNEWT_VAL(CIR_ENABLED)
static bool
cir_complete_cb(struct uwb_dev * udev, struct uwb_mac_interface * cbs)
{
    uint16_t fp_idx, fp_idx_reg;
    dpl_float32_t _rcphase;
    struct cir_dw1000_instance * cir = (struct cir_dw1000_instance *)cbs->inst_ptr;
    struct _dw1000_dev_instance_t *inst = (struct _dw1000_dev_instance_t *)udev;
    struct _dw1000_dev_instance_t *master_inst;

    cir->cir_inst.status.valid = 0;
    CIR_STATS_INC(complete);

    cir->raw_ts = dw1000_read_rawrxtime(cir->dev_inst);
    cir->resampler_delay = dw1000_read_reg(inst, RX_TTCKO_ID, 3, sizeof(uint8_t));
    cir->fp_power = dw1000_get_fppl(inst);

    fp_idx_reg = inst->rxdiag.fp_idx;
    cir->pacc_cnt = inst->rxdiag.pacc_cnt;
    if(!inst->uwb_dev.config.rxdiag_enable) {
        fp_idx_reg = dw1000_read_reg(cir->dev_inst, RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, sizeof(uint16_t));
        cir->pacc_cnt = (dw1000_read_reg(inst, RX_FINFO_ID, 0, sizeof(uint32_t)) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;
    }
    cir->fp_idx = DPL_FLOAT32_DIV(DPL_FLOAT32_I32_TO_F32(fp_idx_reg), DPL_FLOAT32_INIT(64.0f));
    fp_idx = DPL_FLOAT32_INT(DPL_FLOAT32_CEIL(cir->fp_idx));

    master_inst = hal_dw1000_inst(0);
    if (inst->uwb_dev.config.cir_pdoa_slave && master_inst->cir) {
        /* This unit is acting as part of a pdoa network of receivers.
         * instead of trusting the LDE of this unit, use the LDE that detected
         * the earliest first path. This assumes all receivers are within 30mm of
         * each other (or rather their antennas).
         * */

        struct cir_dw1000_instance * master_cir = master_inst->cir;
        dpl_float32_t fp_idx_from_master  = cir_dw1000_remap_fp_index(master_cir, cir);

        /* Check if our first path comes before the master's first path.
         * If so, reread the master's CIR data if possible */
        if (DPL_FLOAT32_INT(fp_idx_from_master) - DPL_FLOAT32_INT(cir->fp_idx) > MYNEWT_VAL(CIR_PDOA_SLAVE_MAX_LEAD)) {
            bool b = cir_dw1000_reread_from_cir(hal_dw1000_inst(0), cir);
            if (!b) {
                return true;
            }
        } else {
            /* Override our local LDE with master's LDE */
            cir->cir_inst.status.lde_override = 1;
            fp_idx = DPL_FLOAT32_INT(DPL_FLOAT32_CEIL(fp_idx_from_master));
        }
    }

    read_from_acc(cir, fp_idx);

    _rcphase = DPL_FLOAT32_I32_TO_F32((int32_t)dw1000_read_reg(cir->dev_inst, RX_TTCKO_ID, 4, sizeof(uint8_t)) & 0x7F);
    cir->rcphase = DPL_FLOAT32_MUL(_rcphase, DPL_FLOAT32_INIT(M_PI/64.0f));

    cir->cir_inst.status.valid = 1;

#if MYNEWT_VAL(CIR_VERBOSE)
    if (inst->rxdiag.rxd.enabled & UWB_RXDIAG_IPATOV_CIR) {
        dpl_eventq_put(dpl_eventq_dflt_get(), &cir->display_event);
    }
#endif
    return false;
}
#endif // MYNEWT_VAL(CIR_ENABLED)

/*!
 * @fn cir_enable(cir_instance_t * inst, bool mode){
 *
 * @brief Enable CIR reading of CIR.
 *
 * @param cir - cir_instance_t *
 * @param mode - bool
 *
 * output parameters
 *
 * returns void
 */
void
cir_dw1000_enable(struct cir_dw1000_instance * cir, bool mode)
{
#if MYNEWT_VAL(CIR_ENABLED)
    dw1000_dev_instance_t * inst = cir->dev_inst;
    cir->cir_inst.status.valid = 0;
    inst->control.cir_enable = mode;
#endif
}

/*!
 * @fn cir_dw1000_get_pdoa(struct cir_dw1000_instance * master, cir_dw1000_instance *slave)
 *
 * @brief Calculate the phase difference between receivers
 *
 * @param master - struct cir_dw1000_instance *
 * @param slave  - struct cir_dw1000_instance *
 *
 * output parameters
 *
 * returns phase_difference - dpl_float32_t
 */
dpl_float32_t
cir_dw1000_get_pdoa(struct cir_dw1000_instance * master, struct cir_dw1000_instance *slave)
{
#ifndef __KERNEL__
    return fmodf((slave->angle - slave->rcphase) - (master->angle - master->rcphase) + 3*M_PI, 2*M_PI) - M_PI;
#else
    dpl_float32_t sd = DPL_FLOAT32_SUB(slave->angle, slave->rcphase);
    dpl_float32_t md = DPL_FLOAT32_SUB(master->angle, master->rcphase);
    dpl_float32_t a = DPL_FLOAT32_ADD(DPL_FLOAT32_SUB(sd, md), DPL_FLOAT32_INIT(3*M_PI));
    dpl_float32_t pdoa = DPL_FLOAT32_FMOD(a, DPL_FLOAT32_INIT(2*M_PI));
    return DPL_FLOAT32_SUB(pdoa, DPL_FLOAT32_INIT(M_PI));
#endif
}

/*!
 * @fn cir_dw1000_init(ir_instance_t * ccp)
 *
 * @brief Allocate resources for the CIR instance.
 *
 * @param inst - struct cir_dw1000_instance *
 *
 * output parameters
 *
 * returns struct cir_dw1000_instance *
 */

struct cir_dw1000_instance *
cir_dw1000_init(struct _dw1000_dev_instance_t * inst, struct cir_dw1000_instance * cir)
{
    if (cir == NULL) {
        cir = (struct cir_dw1000_instance *) calloc(1, sizeof(struct cir_dw1000_instance));
        assert(cir);
        cir->cir_inst.status.selfmalloc = 1;
    }
    cir->dev_inst = inst;
#if MYNEWT_VAL(CIR_VERBOSE)
    dpl_event_init(&cir->display_event, cir_complete_ev_cb, (void*) inst);
#endif

#if MYNEWT_VAL(CIR_STATS)
    int rc = stats_init(
                STATS_HDR(cir->stat),
                STATS_SIZE_INIT_PARMS(cir->stat, STATS_SIZE_32),
                STATS_NAME_INIT_PARMS(cir_dw1000_stat_section)
            );

#if  MYNEWT_VAL(UWB_DEVICE_0) && !MYNEWT_VAL(UWB_DEVICE_1)
    rc |= stats_register("cir", STATS_HDR(cir->stat));
#elif  MYNEWT_VAL(UWB_DEVICE_0) && MYNEWT_VAL(UWB_DEVICE_1)
    if (inst == hal_dw1000_inst(0))
        rc |= stats_register("cir0", STATS_HDR(cir->stat));
    else
        rc |= stats_register("cir1", STATS_HDR(cir->stat));
#endif
    assert(rc == 0);
#endif
    return cir;
}

/*!
 * @fn cir_dw1000_free(struct cir_dw1000_instance * inst)
 *
 * @brief Free resources and restore default behaviour.
 *
 * input parameters
 * @param inst - struct cir_dw1000_instance * inst
 *
 * output parameters
 *
 * returns none
 */
void
cir_dw1000_free(struct cir_dw1000_instance * cir)
{
    assert(cir);
    if (cir->cir_inst.status.selfmalloc) {
        free(cir);
    } else {
        cir->cir_inst.status.initialized = 0;
    }
}

#if MYNEWT_VAL(CIR_ENABLED)
static struct uwb_mac_interface cbs[] = {
    [0] = {
            .id =  UWBEXT_CIR,
            .cir_complete_cb = cir_complete_cb
    },
#if MYNEWT_VAL(UWB_DEVICE_1)
    [1] = {
            .id =  UWBEXT_CIR,
            .cir_complete_cb = cir_complete_cb
    },
#endif
#if MYNEWT_VAL(UWB_DEVICE_2)
    [2] = {
            .id =  UWBEXT_CIR,
            .cir_complete_cb = cir_complete_cb
    }
#endif
};
#endif // MYNEWT_VAL(CIR_ENABLED)

inline static dpl_float32_t
map_cir_dw1000_get_pdoa(struct cir_instance * master, struct cir_instance *slave)
{
    return cir_dw1000_get_pdoa((struct cir_dw1000_instance*)master,
                               (struct cir_dw1000_instance*)slave);
}

inline static void
map_cir_dw1000_enable(struct cir_instance * cir, bool mode)
{
    return cir_dw1000_enable((struct cir_dw1000_instance*)cir, mode);
}

#if MYNEWT_VAL(CIR_ENABLED) && (MYNEWT_VAL(UWB_DEVICE_0) || MYNEWT_VAL(UWB_DEVICE_1) || MYNEWT_VAL(UWB_DEVICE_2))
static const struct cir_driver_funcs cir_dw1000_funcs = {
    .cf_cir_get_pdoa = map_cir_dw1000_get_pdoa,
    .cf_cir_enable = map_cir_dw1000_enable,
};
#endif

/**
 * API to initialise the cir package.
 * @return void
 */
void cir_dw1000_pkg_init(void)
{
#if MYNEWT_VAL(CIR_ENABLED)
    int i;
    struct uwb_dev *udev;
    dw1000_dev_instance_t * inst;
#if MYNEWT_VAL(UWB_PKG_INIT_LOG)
    printf("{\"utime\": %"PRIu32",\"msg\": \"cir_dw1000_pkg_init\"}\n",
           dpl_cputime_ticks_to_usecs(dpl_cputime_get32()));
#endif

    for (i = 0; i < MYNEWT_VAL(UWB_DEVICE_MAX); i++) {
        udev = uwb_dev_idx_lookup(i);
        if (!udev) {
            continue;
        }
        if (udev->device_id != DWT_DEVICE_ID) {
            continue;
        }
        inst = (dw1000_dev_instance_t *)udev;
        cbs[i].inst_ptr = inst->cir = cir_dw1000_init(inst, NULL);
        inst->uwb_dev.cir = (struct cir_instance*)inst->cir;
        inst->cir->cir_inst.cir_funcs = &cir_dw1000_funcs;
        uwb_mac_append_interface(udev, &cbs[i]);
    }
#endif // MYNEWT_VAL(CIR_ENABLED)
}

/**
 * @fn cir_dw1000_pkg_down(void)
 * @brief Uninitialise cir
 *
 * @return void
 */
int cir_dw1000_pkg_down(int reason)
{
    int i;
    struct uwb_dev *udev;
    struct cir_dw1000_instance *cir;

    for (i = 0; i < MYNEWT_VAL(UWB_DEVICE_MAX); i++) {
        udev = uwb_dev_idx_lookup(i);
        if (!udev) {
            continue;
        }
        if (udev->device_id != DWT_DEVICE_ID) {
            continue;
        }
        cir = (struct cir_dw1000_instance*)uwb_mac_find_cb_inst_ptr(udev, UWBEXT_CIR);
        if (!cir) {
            continue;
        }
        uwb_mac_remove_interface(udev, cbs[i].id);
        cir_dw1000_free(cir);
    }

    return 0;
}
