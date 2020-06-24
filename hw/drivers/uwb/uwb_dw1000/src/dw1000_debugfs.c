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
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include "uwbcore.h"
#include <dw1000/dw1000_hal.h>
#include "dw1000_cli_priv.h"
#include <linux/seq_file.h>

#define slog(fmt, ...) \
    pr_info("uwbcore: %s(): %d: "fmt, __func__, __LINE__, ##__VA_ARGS__)

struct debug_cmd {
    char *fn;
    int idx;
};

static struct dentry *dir = 0;
static struct seq_file *seq_file = 0;

static int
buffer_printf(const char *fmt, ...)
{
    va_list args;
    if (!seq_file) return -1;

	va_start(args, fmt);
	seq_vprintf(seq_file, fmt, args);
	va_end(args);
    return 0;
}

static int cmd_dump(struct seq_file *s, void *data)
{
    struct _dw1000_dev_instance_t *inst;
    struct debug_cmd *cmd = (struct debug_cmd*) s->private;
    seq_file = s;
    inst = hal_dw1000_inst(cmd->idx);
    if (!inst) {
        return 0;
    }
    if (!inst->uwb_dev.status.initialized) {
        return 0;
    }

    if (!strcmp(cmd->fn, "dump")) {
        dw1000_cli_dump_registers(inst, buffer_printf);
    }
    if (!strcmp(cmd->fn, "ev")) {
        dw1000_cli_dump_event_counters(inst, buffer_printf);
    }
#if MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN)
    if (!strcmp(cmd->fn, "ibt")) {
        dw1000_cli_interrupt_backtrace(inst, 1, buffer_printf);
    }
#endif
#if MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN) && MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN)
    if (!strcmp(cmd->fn, "bt")) {
        dw1000_cli_backtrace(inst, 1, buffer_printf);
    }
#endif

    seq_file = 0;
	return 0;
}

static int cmd_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmd_dump, inode->i_private);
}

static const struct file_operations clk_dump_fops = {
	.open		= cmd_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static char* cmd_names[] = {
    "dump",
    "ev",
    "ibt",
    "bt",
    0
};

static char* dir_names[] = {
    "dw1000_cli0",
    "dw1000_cli1",
    "dw1000_cli2",
    0
};

static struct debug_cmd cmd_s[ARRAY_SIZE(dir_names)*ARRAY_SIZE(cmd_names)];


void dw1000_debugfs_init(void)
{
    int i, j, k = 0;
    struct _dw1000_dev_instance_t *inst;
    for (i=0;dir_names[i];i++) {
        inst = hal_dw1000_inst(i);
        if (!inst) continue;
        if (!inst->uwb_dev.status.initialized) continue;

        dir = debugfs_create_dir(dir_names[i], uwbcore_get_dfs());
        if (!dir) {
            slog("Failed to create debugfs entry\n");
            continue;
        }

        for (j=0;cmd_names[j];j++) {
            cmd_s[k] = (struct debug_cmd){cmd_names[j], i};
            if (!debugfs_create_file(cmd_names[j], S_IRUGO, dir, &cmd_s[k], &clk_dump_fops)) {
                continue;
            }

            if (++k > ARRAY_SIZE(cmd_s)) {
                pr_err("too few elements in debug_cmd vector\n");
                return;
            }
        }
    }
}

void dw1000_debugfs_deinit(void)
{
    if (dir) {
        debugfs_remove_recursive(dir);
    }
}
#endif
