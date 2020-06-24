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
 * @file dw1000_pkg.c
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief package file
 *
 * @details This is the pkg class which utilises the function to initialize the dw1000 instances depending on the availability.
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <dpl/dpl.h>
#include <dpl/dpl_cputime.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_phy.h>

int dw1000_cli_register(void);
int dw1000_cli_down(int reason);

#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

/**
 * API to initialize the dw1000 instances.
 *
 * @param void
 * @return void
 */
void dw1000_pkg_init(void)
{
#if defined(MYNEWT)

#if MYNEWT_VAL(UWB_PKG_INIT_LOG)
    DIAGMSG("{\"utime\": %lu,\"msg\": \"dw1000_pkg_init\"}\n", dpl_cputime_ticks_to_usecs(dpl_cputime_get32()));
#endif

#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_dev_config(hal_dw1000_inst(0));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    dw1000_dev_config(hal_dw1000_inst(1));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    dw1000_dev_config(hal_dw1000_inst(2));
#endif

#else
    struct os_dev *dev;

    dev = os_dev_lookup("dw1000_0");
    if (dev) {
        dw1000_dev_config((struct _dw1000_dev_instance_t*)dev);
    }
    dev = os_dev_lookup("dw1000_1");
    if (dev) {
        dw1000_dev_config((struct _dw1000_dev_instance_t*)dev);
    }
    dev = os_dev_lookup("dw1000_2");
    if (dev) {
        dw1000_dev_config((struct _dw1000_dev_instance_t*)dev);
    }
#endif

#if MYNEWT_VAL(DW1000_CLI)
    dw1000_cli_register();
#endif
}

int dw1000_pkg_down(int reason)
{
    struct os_dev *dev;
#if MYNEWT_VAL(UWB_PKG_INIT_LOG)
    DIAGMSG("{\"utime\": %"PRIu32",\"msg\": \"dw1000_pkg_down\"}\n", dpl_cputime_ticks_to_usecs(dpl_cputime_get32()));
#endif

    dev = os_dev_lookup("dw1000_0");
    if (dev) {
        dw1000_dev_deinit((struct _dw1000_dev_instance_t *)dev);
    }
    dev = os_dev_lookup("dw1000_1");
    if (dev) {
        dw1000_dev_deinit((struct _dw1000_dev_instance_t *)dev);
    }
    dev = os_dev_lookup("dw1000_2");
    if (dev) {
        dw1000_dev_deinit((struct _dw1000_dev_instance_t *)dev);
    }
#if MYNEWT_VAL(DW1000_CLI)
    dw1000_cli_down(reason);
#endif
    return 0;
}
