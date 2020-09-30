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

#include <syscfg/syscfg.h>

#ifdef __KERNEL__
#include <dpl/dpl.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include "uwbcore.h"
#include <dw1000/dw1000_hal.h>
#include "dw1000_cli_priv.h"

#define slog(fmt, ...) \
    pr_info("uwbcore: %s(): %d: "fmt, __func__, __LINE__, ##__VA_ARGS__)

struct dw1000cli_sysfs_data {
    struct kobject *kobj;
    char device_name[16];
    struct mutex local_lock;
    struct _dw1000_dev_instance_t *inst;

    struct kobj_attribute *dev_attr_cfg;
    struct attribute** attrs;
    struct attribute_group attribute_group;
};

static struct dw1000cli_sysfs_data dw1000cli_sysfs_inst[MYNEWT_VAL(UWB_DEVICE_MAX)] = {0};

static struct dw1000cli_sysfs_data* get_instance(struct kobject* kobj)
{
    struct kobject* ko = kobj;
    /* Walk up in parent objects until they end with a number we can use
     * to figure out our instance */
    while (ko) {
        char s = ko->name[strlen(ko->name)-1];
        if (s >= '0' && s<= '9') {
            u8 i = s-'0';
            if (i<MYNEWT_VAL(UWB_DEVICE_MAX)) {
                return &dw1000cli_sysfs_inst[i];
            }
        }
        ko = ko->parent;
    }
    return 0;
}

static ssize_t cmd_show(struct kobject *kobj,
    struct kobj_attribute *attr, char *buf)
{
    int gpio_num;
    unsigned int copied;
    const char nodev_errmsg[] = "err, no device\n";
    struct dw1000cli_sysfs_data *ed;
    struct _dw1000_dev_instance_t *inst;

    ed = get_instance(kobj);
    if (!ed) {
        slog("ERROR: Could not get instance, kobj->name = '%s'\n", kobj->name);
        return -EINVAL;
    }
    inst = ed->inst;

    if (!inst->uwb_dev.status.initialized) {
        memcpy(buf, nodev_errmsg, strlen(nodev_errmsg));
        return strlen(nodev_errmsg);
    }

    if (!strcmp(attr->attr.name, "device_id")) {
        copied = snprintf(buf, PAGE_SIZE, "0x%08X\n", inst->uwb_dev.device_id);
    }

    if (!strcmp(attr->attr.name, "addr") ||
        !strcmp(attr->attr.name, "uid")) {
        copied = snprintf(buf, PAGE_SIZE, "0x%04X\n", inst->uwb_dev.uid);
    }

    if (!strcmp(attr->attr.name, "euid")) {
        copied = snprintf(buf, PAGE_SIZE, "0x%016llX\n", inst->uwb_dev.euid);
    }

    if (!strncmp(attr->attr.name, "gpio", 4)) {
        gpio_num = attr->attr.name[4] - '0';
        slog("gpio %d", gpio_num);
        if (!strncmp(attr->attr.name+5, "_mode", 5)) {
            snprintf(buf, PAGE_SIZE, "%d\n", dw1000_gpio_get_mode(inst, gpio_num));
        } else if (!strncmp(attr->attr.name+5, "_dir", 5)) {
            /* Invert value as 1=input, 0=output for dwX000 */
            snprintf(buf, PAGE_SIZE, "%d\n", !dw1000_gpio_get_direction(inst, gpio_num));
        } else {
            snprintf(buf, PAGE_SIZE, "%d\n", dw1000_gpio_read(inst, gpio_num));
        }
    }

    return copied;
}

static ssize_t cmd_store(struct kobject *kobj,
    struct kobj_attribute *attr, const char *buf, size_t count)
{
    int gpio_num;
    int ret;
    u64 res;
    struct dw1000cli_sysfs_data *ed;
    struct _dw1000_dev_instance_t *inst;

    ed = get_instance(kobj);
    if (!ed) {
        slog("ERROR: Could not get instance, kobj->name = '%s'\n", kobj->name);
        return -EINVAL;
    }
    inst = ed->inst;

    if (!inst->uwb_dev.status.initialized) {
        return count;
    }

    if (!strcmp(attr->attr.name, "addr") ||
        !strcmp(attr->attr.name, "uid")) {
        ret = kstrtoll(buf, 0, &res);
        if (!ret) {
            inst->uwb_dev.uid = res;
            uwb_set_uid(&inst->uwb_dev,
                        inst->uwb_dev.uid);
        }
    }

    if (!strcmp(attr->attr.name, "euid")) {
        ret = kstrtoll(buf, 0, &res);
        if (!ret) {
            inst->uwb_dev.euid = res;
            uwb_set_euid(&inst->uwb_dev,
                         inst->uwb_dev.euid);
        }
    }

    if (!strcmp(attr->attr.name, "cw")) {
        dw1000_write_reg(inst, SYS_MASK_ID, 0, 0, sizeof(uint32_t));
        dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_TRXOFF, sizeof(uint8_t));
        dw1000_configcwmode(inst, inst->uwb_dev.config.channel);
        slog("Device now in CW mode on ch %d. Reset to continue\n",
             inst->uwb_dev.config.channel);

    }

#if MYNEWT_VAL(DW1000_CLI_EVENT_COUNTERS)
    if (!strcmp(attr->attr.name, "ev")) {
        ret = kstrtoll(buf, 0, &res);
        if (!ret) {
            dw1000_phy_event_cnt_ctrl(inst, (res != 0), true);
        }
    }
#endif

    if (!strncmp(attr->attr.name, "gpio", 4)) {
        gpio_num = attr->attr.name[4] - '0';
        ret = kstrtoll(buf, 0, &res);
        if (ret) {
            return -EINVAL;
        }
        if (!strncmp(attr->attr.name+5, "_mode", 5) && res < 5) {
            slog("gpio %d mode %"PRIu64, gpio_num, res);
            dw1000_gpio_set_mode(inst, gpio_num, res);
        } else if (!strncmp(attr->attr.name+5, "_dir", 5) && res < 2) {
            /* Invert value as 1=input, 0=output for dwX000 */
            dw1000_gpio_set_direction(inst, gpio_num, (res==0));
            slog("gpio(%d): %s\n", gpio_num, (res==0)?"in":"out");
        } else {
            slog("gpio %d write %"PRIu64, gpio_num, res);
            dw1000_gpio_init_out(inst, gpio_num, res);
        }
    }

    return count;
}


#define DEV_ATTR_ROW(__X)                                               \
    (struct kobj_attribute){                                            \
        .attr = {.name = __X,                                           \
                 .mode = VERIFY_OCTAL_PERMISSIONS(S_IRUGO)}, .show = cmd_show}

#define DEV_ATTR_ROW_W(__X)                                             \
    (struct kobj_attribute){                                            \
        .attr = {.name = __X,                                           \
                 .mode = VERIFY_OCTAL_PERMISSIONS(S_IRUGO|S_IWUSR|S_IWGRP)}, \
            .show = cmd_show, .store = cmd_store}

/* Read only interface(s) */
static const char* ro_attr[] = {
    "device_id",
};

/* Read-Write interfaces */
static const char* rw_attr[] = {
    "addr",
    "uid",
    "euid",
    "cw",
#if MYNEWT_VAL(DW1000_CLI_EVENT_COUNTERS)
    "ev",
#endif
    "gpio0_mode", "gpio1_mode", "gpio2_mode",
    "gpio3_mode", "gpio4_mode", "gpio5_mode",
    "gpio6_mode", "gpio7_mode", "gpio8_mode",
    "gpio0_dir", "gpio1_dir", "gpio2_dir",
    "gpio3_dir", "gpio4_dir", "gpio5_dir",
    "gpio6_dir", "gpio7_dir", "gpio8_dir",
    "gpio0", "gpio1", "gpio2",
    "gpio3", "gpio4", "gpio5",
    "gpio6", "gpio7", "gpio8"
};

int dw1000_sysfs_init(struct _dw1000_dev_instance_t *inst)
{
    int ret, i, k;
    struct dw1000cli_sysfs_data *ed;
    if (inst->uwb_dev.idx >= ARRAY_SIZE(dw1000cli_sysfs_inst)) {
        return -EINVAL;
    }
    ed = &dw1000cli_sysfs_inst[inst->uwb_dev.idx];
    ed->inst = inst;

    snprintf(ed->device_name, sizeof(ed->device_name)-1, "dw1000_cli%d", inst->uwb_dev.idx);
    ed->kobj = kobject_create_and_add(ed->device_name, uwbcore_get_kobj());
    if (!ed->kobj) {
        slog("Failed to create %s\n", ed->device_name);
        return -ENOMEM;
    }

    ed->dev_attr_cfg = kzalloc((ARRAY_SIZE(ro_attr) + ARRAY_SIZE(rw_attr))*sizeof(struct kobj_attribute), GFP_KERNEL);
    if (!ed->dev_attr_cfg) {
        slog("Failed to create dev_attr_cfg\n");
        goto err_dev_attr_cfg;
    }

    ed->attrs = kzalloc((ARRAY_SIZE(ro_attr) + ARRAY_SIZE(rw_attr) + 1)*sizeof(struct attribute *), GFP_KERNEL);
    if (!ed->attrs) {
        slog("Failed to create sysfs_attrs\n");
        goto err_sysfs_attrs;
    }

    k=0;
    for (i=0;i<ARRAY_SIZE(ro_attr);i++) {
        ed->dev_attr_cfg[k] = DEV_ATTR_ROW(ro_attr[i]);
        ed->attrs[k] = &ed->dev_attr_cfg[k].attr;
        k++;
    }
    for (i=0;i<ARRAY_SIZE(rw_attr);i++) {
        ed->dev_attr_cfg[k] = DEV_ATTR_ROW_W(rw_attr[i]);
        ed->attrs[k] = &ed->dev_attr_cfg[k].attr;
        k++;
    }

    ed->attribute_group.attrs = ed->attrs;
    ret = sysfs_create_group(ed->kobj, &ed->attribute_group);
    if (ret) {
        slog("Failed to create sysfs group\n");
        goto err_sysfs;
    }

    return 0;

err_sysfs:
    kfree(ed->attrs);
err_sysfs_attrs:
    kfree(ed->dev_attr_cfg);
err_dev_attr_cfg:
    kobject_put(ed->kobj);
    return -ENOMEM;
}

void dw1000_sysfs_deinit(int idx)
{
    struct dw1000cli_sysfs_data *ed;
    if (idx >= ARRAY_SIZE(dw1000cli_sysfs_inst)) {
        return;
    }
    ed = &dw1000cli_sysfs_inst[idx];

    if (ed->kobj) {
        mutex_destroy(&ed->local_lock);
        sysfs_remove_group(ed->kobj, &ed->attribute_group);
        kfree(ed->attrs);
        kfree(ed->dev_attr_cfg);
        kobject_put(ed->kobj);
    }
}
#endif
