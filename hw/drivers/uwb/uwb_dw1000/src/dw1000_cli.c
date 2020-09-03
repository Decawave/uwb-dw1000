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
 * @file dw1000_cli.c
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief Command debug interface
 *
 * @details
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <dpl/dpl_cputime.h>
#include <hal/hal_gpio.h>

#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_regs.h>
#include "dw1000_cli_priv.h"

#if MYNEWT_VAL(DW1000_CLI)

#include <shell/shell.h>
#include <console/console.h>
#include <console/ticks.h>
#if MYNEWT_VAL(UWB_CCP_ENABLED)
#include <uwb_ccp/uwb_ccp.h>
#endif
#if MYNEWT_VAL(UWB_RNG_ENABLED)
#include <uwb_rng/uwb_rng.h>
#endif

#ifdef __KERNEL__
#ifndef console_printf
int console_printf(const char* fmt, ...) { return 0; }
#endif
int dw1000_sysfs_init(struct _dw1000_dev_instance_t *inst);
void dw1000_sysfs_deinit(int idx);
void dw1000_debugfs_init(void);
void dw1000_debugfs_deinit(void);
#endif

static int dw1000_cli_cmd(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *streamer);

#if MYNEWT_VAL(SHELL_CMD_HELP)
const struct shell_param cmd_dw1000_param[] = {
    {"dump", "[inst] dump all registers"},
#if MYNEWT_VAL(DW1000_CLI_EVENT_COUNTERS)
    {"ev", "<inst> <on|reset|dump> event counters"},
#endif
    {"cw", "<inst> tx CW on current channel"},
    {"da", "<inst> <addr> [length], dump area"},
    {"rd", "<inst> <addr> <subaddr> <length>, read register"},
    {"wr", "<inst> <addr> <subaddr> <value> <length>, write value to register"},
#if MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN)
    {"ibt", "[instance [verbose-num]] interrupt backtrace"},
    {"status2txt", "<sys_status> to text"},
    {"fctrl2txt", "<fctrl> to text"},
#endif
#if MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN)
    {"spibt", "[instance [verbose-num]] spi backtrace"},
#endif
#if MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN) && MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN)
    {"bt", "[instance [verbose-num]] spi+irq backtrace"},
#endif
    {NULL,NULL},
};

const struct shell_cmd_help cmd_dw1000_help = {
	"dw1000 dbg", "dw1000 debug", cmd_dw1000_param
};
#endif

static struct shell_cmd shell_dw1000_cmd =
    SHELL_CMD_EXT("dw1000", dw1000_cli_cmd, &cmd_dw1000_help);

void
dw1000_cli_dump_registers(struct _dw1000_dev_instance_t * inst, struct streamer *streamer)
{
    uint64_t reg = 0;
    int i, l;

    for(i=0; i<0x37; i++)
    {
        if (i==0x05 || i==0x07 || i==0x0B ||
            i==0x16 || i==0x1B || i==0x1C ||
            i==0x20 || i==0x22 || i==0x29 ||
            (i>0x29 && i<0x36)) {
            continue;
        }
        switch (i) {
        case (DEV_ID_ID):
        case (PANADR_ID):
        case (SYS_CFG_ID):
        case (SYS_CTRL_ID):
        case (RX_FINFO_ID):
        case (RX_TTCKI_ID):
        case (ACK_RESP_T_ID):
        case (RX_SNIFF_ID):
        case (TX_POWER_ID):
        case (CHAN_CTRL_ID):
        case (TX_ANTD_ID):
        case (RX_FWTO_ID):
            reg = dw1000_read_reg(inst, i, 0, 4);
            streamer_printf(streamer, "{\"reg[%02X]\"=\"0x%08llX\"}\n",i,reg&0xffffffff);
            break;
        case (SYS_TIME_ID):
        case (TX_FCTRL_ID):
        case (DX_TIME_ID):
        case (SYS_STATUS_ID):
        case (RX_TTCKO_ID):
        case (RX_TIME_ID):
        case (TX_TIME_ID):
        case (SYS_MASK_ID):
        case (SYS_STATE_ID):
            reg = dw1000_read_reg(inst, i, 0, 5);
            streamer_printf(streamer, "{\"reg[%02X]\"=\"0x%010llX\"}\n",i,reg&0xffffffffffll);
            break;
        default:
            l=8;
            reg = dw1000_read_reg(inst, i, 0, l);
            streamer_printf(streamer, "{\"reg[%02X]\"=\"0x%016llX\"}\n",i,
                           reg&0xffffffffffffffffll);
        }
    }
    streamer_printf(streamer, "{\"inst->irq_sem\"=%d}\n", dpl_sem_get_count(&inst->uwb_dev.irq_sem));
    streamer_printf(streamer, "{\"inst->tx_sem\"=%d}\n", dpl_sem_get_count(&inst->tx_sem));
#if MYNEWT_VAL(UWB_RNG_ENABLED)
    struct uwb_rng_instance *rng = (struct uwb_rng_instance*)uwb_mac_find_cb_inst_ptr(&inst->uwb_dev, UWBEXT_RNG);
    if (rng)
        streamer_printf(streamer, "{\"rng->sem\"=%d}\n", dpl_sem_get_count(&rng->sem));
#endif

#if defined(MYNEWT)
#if MYNEWT_VAL(UWB_CCP_ENABLED)
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(&inst->uwb_dev, UWBEXT_CCP);
    if (ccp)
        streamer_printf(streamer, "{\"ccp->sem\"=%d}\n", dpl_sem_get_count(&ccp->sem));
#endif
#endif
}

void
dw1000_cli_dump_address(struct _dw1000_dev_instance_t * inst, uint32_t addr, uint16_t length, struct streamer *streamer)
{
#define DUMP_STEP (16)
    int i, step = DUMP_STEP;
    uint8_t b[DUMP_STEP];
    streamer_printf(streamer, "Dump starting at %06"PRIX32":\n", addr);
    for (i=0;i<length;i+=step) {
        memset(b,0,sizeof(b));
        dw1000_read(inst, addr, i, b, step);

        streamer_printf(streamer, "%04X: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
               i, b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
               b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
    }
}

#if MYNEWT_VAL(DW1000_CLI_EVENT_COUNTERS)
void
dw1000_cli_dump_event_counters(struct _dw1000_dev_instance_t * inst, struct streamer *streamer)
{
    struct uwb_dev_evcnt cnt;

    dw1000_phy_event_cnt_read(inst, &cnt);

    streamer_printf(streamer, "Event counters:\n");
    streamer_printf(streamer, "  RXPHE:  %6d  # rx PHR err\n", cnt.ev0s.count_rxphe);
    streamer_printf(streamer, "  RXFSL:  %6d  # rx frame sync loss\n", cnt.ev0s.count_rxfsl);
    streamer_printf(streamer, "  RXFCG:  %6d  # rx CRC OK\n", cnt.ev1s.count_rxfcg);
    streamer_printf(streamer, "  RXFCE:  %6d  # rx CRC Errors\n", cnt.ev1s.count_rxfce);
    streamer_printf(streamer, "  ARFE:   %6d  # addr filt err\n", cnt.ev2s.count_arfe);
    streamer_printf(streamer, "  RXOVRR: %6d  # rx overflow\n", cnt.ev2s.count_rxovrr);
    streamer_printf(streamer, "  RXSTO:  %6d  # rx SFD TO\n", cnt.ev3s.count_rxsto);
    streamer_printf(streamer, "  RXPTO:  %6d  # pream search TO\n", cnt.ev3s.count_rxpto);
    streamer_printf(streamer, "  FWTO:   %6d  # rx frame wait TO\n", cnt.ev4s.count_fwto);
    streamer_printf(streamer, "  TXFRS:  %6d  # tx frames sent\n", cnt.ev4s.count_txfrs);
    streamer_printf(streamer, "  HPWARN: %6d  # half period warn\n", cnt.ev5s.count_hpwarn);
    streamer_printf(streamer, "  TPW:    %6d  # tx pwr-up warn\n", cnt.ev5s_1000.count_tpwarn);
}
#endif

#if MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN)
static char*
sys_status_to_string(uint64_t s)
{
    static char b[128];
    memset(b,0,sizeof(b));
    char *bp = b;
    if (s & 0x0400000000) bp += snprintf(bp,sizeof(b)-(bp-b), "TxPwrUpTmErr|" );
    if (s & 0x0200000000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxPreambRej|");
    if (s & 0x0100000000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxReedSolCorr|");
    if (s & 0x80000000) bp += snprintf(bp,sizeof(b)-(bp-b), "ICRxBufPtr=1|");
    if (s & 0x40000000) bp += snprintf(bp,sizeof(b)-(bp-b), "HostRxBufPtr=1|");
    if (s & 0x20000000) bp += snprintf(bp,sizeof(b)-(bp-b), "AutFrameFiltRej|");
    if (s & 0x10000000) bp += snprintf(bp,sizeof(b)-(bp-b), "TransmitBufferError|");
    if (s & 0x08000000) bp += snprintf(bp,sizeof(b)-(bp-b), "HalfPeriodDelayWarn|");
    if (s & 0x04000000) bp += snprintf(bp,sizeof(b)-(bp-b), "RXSFDTimeout|");
    if (s & 0x02000000) bp += snprintf(bp,sizeof(b)-(bp-b), "ClockPLLLosingLock|");
    if (s & 0x01000000) bp += snprintf(bp,sizeof(b)-(bp-b), "RFPLLLosingLock|");
    if (s & 0x00800000) bp += snprintf(bp,sizeof(b)-(bp-b), "SLEEP2INIT|");
    if (s & 0x00400000) bp += snprintf(bp,sizeof(b)-(bp-b), "GpioInt|");
    if (s & 0x00200000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxPreamDetTimeout|");
    if (s & 0x00100000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxOvErr|");
    if (s & 0x00080000) bp += snprintf(bp,sizeof(b)-(bp-b), "(bit19 reserved)|");
    if (s & 0x00040000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxLDEerr|");
    if (s & 0x00020000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxTimeout|");
    if (s & 0x00010000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxReedSolomonFrameSyncLoss|");
    if (s & 0x00008000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxFCSErr|");
    if (s & 0x00004000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxFCSGood|");
    if (s & 0x00002000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxDataFrmRdy|");
    if (s & 0x00001000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxPHYErr|");
    if (s & 0x00000800) bp += snprintf(bp,sizeof(b)-(bp-b), "RxPHYDet|");
    if (s & 0x00000400) bp += snprintf(bp,sizeof(b)-(bp-b), "RxLDEdone|");
    if (s & 0x00000200) bp += snprintf(bp,sizeof(b)-(bp-b), "RxSFDet|");
    if (s & 0x00000100) bp += snprintf(bp,sizeof(b)-(bp-b), "RxPreamDet|");
    if (s & 0x00000080) bp += snprintf(bp,sizeof(b)-(bp-b), "TxFrameSent|");
    if (s & 0x00000040) bp += snprintf(bp,sizeof(b)-(bp-b), "TxPHYDone|");
    if (s & 0x00000020) bp += snprintf(bp,sizeof(b)-(bp-b), "TxPreamDone|");
    if (s & 0x00000010) bp += snprintf(bp,sizeof(b)-(bp-b), "TxStart|");
    if (s & 0x00000008) bp += snprintf(bp,sizeof(b)-(bp-b), "AutoAck|");
    if (s & 0x00000004) bp += snprintf(bp,sizeof(b)-(bp-b), "ExtClock Reset|");
    if (s & 0x00000002) bp += snprintf(bp,sizeof(b)-(bp-b), "Clock PLL Lock|");
    if (s & 0x00000001) bp += snprintf(bp,sizeof(b)-(bp-b), "IRS");
    return b;
}

static char*
fctrl_to_string(uint16_t s)
{
    static char b[40];
    char *bp = b;
    memset(b,0,sizeof(b));
    if ((s & 0x0007) == 0x0001) bp += snprintf(bp,sizeof(b)-(bp-b), "D|");  /* Data */
    if ((s & 0x0007) == 0x0002) bp += snprintf(bp,sizeof(b)-(bp-b), "A|");  /* Acknowledge */
    if ((s & 0x0007) == 0x0003) bp += snprintf(bp,sizeof(b)-(bp-b), "M|");  /* Reserved */
    if ((s & 0x0007) == 0x0004) bp += snprintf(bp,sizeof(b)-(bp-b), "R|");
    if ((s & 0x0007) == 0x0005) bp += snprintf(bp,sizeof(b)-(bp-b), "R|");
    if ((s & 0x0007) == 0x0006) bp += snprintf(bp,sizeof(b)-(bp-b), "R|");
    if ((s & 0x0007) == 0x0007) bp += snprintf(bp,sizeof(b)-(bp-b), "R|");

    if ((s & 0x0008) == 0x0008) bp += snprintf(bp,sizeof(b)-(bp-b), "Secr|"); /* Security Enabeled */
    if ((s & 0x0010) == 0x0010) bp += snprintf(bp,sizeof(b)-(bp-b), "fPnd|"); /* Frame Pending */
    if ((s & 0x0020) == 0x0020) bp += snprintf(bp,sizeof(b)-(bp-b), "ACKr|"); /* Ack requested */
    if ((s & 0x0040) == 0x0040) bp += snprintf(bp,sizeof(b)-(bp-b), "PANc|"); /* PANID Compress */

    if ((s & 0x0C00) == 0x0000) bp += snprintf(bp,sizeof(b)-(bp-b), "DstNo|"); /* No destination address */
    if ((s & 0x0C00) == 0x0400) bp += snprintf(bp,sizeof(b)-(bp-b), "DstRs|"); /* Reserved */
    if ((s & 0x0C00) == 0x0800) bp += snprintf(bp,sizeof(b)-(bp-b), "Dst16|"); /* 16-bit destination address */
    if ((s & 0x0C00) == 0x0C00) bp += snprintf(bp,sizeof(b)-(bp-b), "Dst64|"); /* 64-bit destination address */

    if ((s & 0x3000) == 0x0000) bp += snprintf(bp,sizeof(b)-(bp-b), "I2003|");
    if ((s & 0x3000) == 0x1000) bp += snprintf(bp,sizeof(b)-(bp-b), "I|");
    if ((s & 0x3000) == 0x2000) bp += snprintf(bp,sizeof(b)-(bp-b), "iFv|"); /* Invalid frame version */
    if ((s & 0x3000) == 0x3000) bp += snprintf(bp,sizeof(b)-(bp-b), "iFv|");

    if ((s & 0xC000) == 0x0000) bp += snprintf(bp,sizeof(b)-(bp-b), "SrcNo"); /* No destination address */
    if ((s & 0xC000) == 0x4000) bp += snprintf(bp,sizeof(b)-(bp-b), "SrcRs"); /* Reserved */
    if ((s & 0xC000) == 0x8000) bp += snprintf(bp,sizeof(b)-(bp-b), "Src16"); /* 16-bit destination address */
    if ((s & 0xC000) == 0xC000) bp += snprintf(bp,sizeof(b)-(bp-b), "Src64"); /* 64-bit destination address */

    return b;
}

static void
fctrl_ledgend(struct streamer *streamer)
{
    streamer_printf(streamer, "   D=Data, A=Ack, M=Mac\n");
    streamer_printf(streamer, "   Secr: Security enabled, fPnd: Frame pending, ACKr: Ack requested, PANc: PANID Compress\n");
    streamer_printf(streamer, "   Dst: No=no dest addres, Rs=Reserved, 16-bit address, 64-bit address\n");
    streamer_printf(streamer, "   Frame version: I-IEEE 802.15.4, I2003-IEEE 802.15.4-2003, iFv-Invalid Frame Version\n");
    streamer_printf(streamer, "   Src: No=no src addres, Rs=Reserved, 16-bit address, 64-bit address\n");
}

static int
print_interrupt_bt_line(uint32_t *start_t, uint16_t verbose,
                        struct dw1000_sys_status_backtrace *p,
                        struct dw1000_sys_status_backtrace *p_last,
                        struct streamer *streamer)
{
    if (!p->utime) return 0;
    if (!*start_t) *start_t = p->utime;

    int32_t diff = (p_last)? p->utime-p_last->utime : 0;
    if (diff < 0) diff = 0;
    streamer_printf(streamer, " %10lu ", dpl_cputime_ticks_to_usecs(p->utime));
    streamer_printf(streamer, " %10lu ", dpl_cputime_ticks_to_usecs(p->utime-*start_t));
    streamer_printf(streamer, " %8lu ", dpl_cputime_ticks_to_usecs(diff));
    streamer_printf(streamer, " %6lu ", dpl_cputime_ticks_to_usecs(p->utime_end-p->utime));
    streamer_printf(streamer, " %2s ", p->interrupt_reentry ? "r":" ");
    if (p->fctrl) {
        if (verbose&0x1) {
            char *s = fctrl_to_string(p->fctrl);
            streamer_printf(streamer, " %02X %02X (%s)%*s", p->fctrl&0xff, p->fctrl>>8, s, 32-strlen(s), " ");
        } else {
            streamer_printf(streamer, " %02X %02X ", p->fctrl&0xff, p->fctrl>>8);
        }
    } else {
        streamer_printf(streamer, "       ");
        if (verbose&0x1) {
            streamer_printf(streamer, " %32s ", "");
        }
    }
    streamer_printf(streamer, " %0*llX ", 2*DW1000_SYS_STATUS_ASSEMBLE_LEN, DW1000_SYS_STATUS_ASSEMBLE(p));
    streamer_printf(streamer, " %s", sys_status_to_string(DW1000_SYS_STATUS_ASSEMBLE(p)));
    return 1;
}

void
dw1000_cli_interrupt_backtrace(struct _dw1000_dev_instance_t * inst, uint16_t verbose, struct streamer *streamer)
{
    int i;
    uint32_t start_t = 0;
    struct dw1000_sys_status_backtrace *p, *p_last=0;

    streamer_printf(streamer, " %10s ", "abs");
    streamer_printf(streamer, " %10s ", "usec");
    streamer_printf(streamer, " %8s ", "diff");
    streamer_printf(streamer, " %6s ", "dur");
    streamer_printf(streamer, " %2s", "ir");
    streamer_printf(streamer, " %5s", "fctrl");
    if (verbose&0x1) {
        streamer_printf(streamer, "(fctrl2txt)%21s ", "");
    }
    streamer_printf(streamer, " %*s ", 2*DW1000_SYS_STATUS_ASSEMBLE_LEN, "status");
    streamer_printf(streamer, "   status2txt\n");
    for (i=0;i<80;i++) streamer_printf(streamer, "-");
    if (verbose&0x1) {
        for (i=0;i<34;i++) streamer_printf(streamer, "-");
    }
    streamer_printf(streamer, "\n");


    inst->sys_status_bt_lock = 1;
    for (i=0;i<MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN);i++) {
        uint16_t i_mod = (inst->sys_status_bt_idx + i + 1) % MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN);
        p = &inst->sys_status_bt[i_mod];
        if (print_interrupt_bt_line(&start_t, verbose, p, p_last, streamer)) {
            streamer_printf(streamer, "\n");
        }
        p_last = p;
    }
    inst->sys_status_bt_lock = 0;

    if (verbose&0x1) {
        streamer_printf(streamer, "----\n fctrl2txt: \n");
        fctrl_ledgend(streamer);
    }
}
#endif

#if MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN)
static char*
cmd_to_string(uint8_t *cmd, uint8_t cmd_len)
{
    static char b[16];
    memset(b,0,sizeof(b));
    char *bp = b;
    if (cmd[0] & 0x80) bp += snprintf(bp,sizeof(b)-(bp-b), "W");
    else               bp += snprintf(bp,sizeof(b)-(bp-b), "R");
    uint8_t cmd_reg  = cmd[0]&0x3F;
    bp += snprintf(bp,sizeof(b)-(bp-b), "@%02X", cmd_reg);
    if (cmd_len > 1) {
        uint8_t cmd_extended = (0x8 & cmd[1]) != 0;
        uint16_t subaddress = 0x7F & cmd[1];
        if (cmd_extended && cmd_len > 2) {
            subaddress |= ((uint16_t)cmd[2]) << 7;
        }
        bp += snprintf(bp,sizeof(b)-(bp-b), ":%x", subaddress);
    }

    return b;
}

static int
print_spi_bt_line(uint32_t *start_t, uint16_t verbose,
                  struct dw1000_spi_backtrace *p,
                  struct dw1000_spi_backtrace *p_last,
                  struct streamer *streamer)
{
    int j;
    if (!p->utime) return 0;
    if (!*start_t) *start_t = p->utime;

    int32_t diff = (p_last)? p->utime-p_last->utime : 0;
    if (diff < 0) diff = 0;
    streamer_printf(streamer, " %10lu ", dpl_cputime_ticks_to_usecs(p->utime));
    streamer_printf(streamer, " %10lu ", dpl_cputime_ticks_to_usecs(p->utime-*start_t));
    streamer_printf(streamer, " %8lu ", dpl_cputime_ticks_to_usecs(diff));
    streamer_printf(streamer, " %6lu ", dpl_cputime_ticks_to_usecs(p->utime_end-p->utime));
    streamer_printf(streamer, " %3s%s  ", (p->non_blocking)?"NB-":"", (p->is_write)?"W":"R");

    streamer_printf(streamer, " ");
    for (j=0;j<4;j++) {
        if (j<p->cmd_len) {
            streamer_printf(streamer, "%02X", p->cmd[j]);
        } else {
            streamer_printf(streamer, "  ");
        }
    }

    if (verbose&0x1) {
        char *s = cmd_to_string(p->cmd, p->cmd_len);
        streamer_printf(streamer, " (%s)%*s", s, 17-strlen(s), " ");
    }
    streamer_printf(streamer, " %4d ", p->data_len);
    for (j=0;j<p->data_len&&j<sizeof(p->data);j++) {
        streamer_printf(streamer, "%02X", p->data[j]);
    }
    return 1;
}

void
dw1000_cli_spi_backtrace(struct _dw1000_dev_instance_t * inst, uint16_t verbose,
                         struct streamer *streamer)
{
    int i;
    uint32_t start_t = 0;
    struct dw1000_spi_backtrace *p, *p_last=0;

    streamer_printf(streamer, " %10s ", "abs");
    streamer_printf(streamer, " %10s ", "usec");
    streamer_printf(streamer, " %8s ", "diff");
    streamer_printf(streamer, " %6s ", "dur");
    streamer_printf(streamer, " %5s", "flags");
    streamer_printf(streamer, " %s      ", "cmd");
    if (verbose&0x1) {
        streamer_printf(streamer, "(cmd2txt)%21s ", "");
    }
    streamer_printf(streamer, " %4s ", "dlen");
    streamer_printf(streamer, " %s\n", "data");
    for (i=0;i<80;i++) streamer_printf(streamer, "-");
    if (verbose&0x1) {
        for (i=0;i<34;i++) streamer_printf(streamer, "-");
    }
    streamer_printf(streamer, "\n");

    inst->spi_bt_lock = 1;
    for (i=0;i<MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN);i++) {
        uint16_t i_mod = (inst->spi_bt_idx + i + 1) % MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN);
        p = &inst->spi_bt[i_mod];
        if (print_spi_bt_line(&start_t, verbose, p, p_last, streamer)) {
            streamer_printf(streamer, "\n");
        }
        p_last = p;
    }
    inst->spi_bt_lock = 0;
}
#endif

#if MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN) && MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN)

void
dw1000_cli_backtrace(struct _dw1000_dev_instance_t * inst, uint16_t verbose, struct streamer *streamer)
{
    int i;
    uint16_t spi_i=0, irq_i=0;
    uint32_t start_t = 0;
    struct dw1000_spi_backtrace *spi_p, *spi_p_last=0;
    struct dw1000_sys_status_backtrace *irq_p, *irq_p_last=0;

    streamer_printf(streamer, " %10s ", "abs");
    streamer_printf(streamer, " %10s ", "usec");
    streamer_printf(streamer, " %8s ", "diff");
    streamer_printf(streamer, " %6s ", "dur");
    streamer_printf(streamer, " %5s", "flags");
    streamer_printf(streamer, " cmd/status data\n");
    for (i=0;i<80;i++) streamer_printf(streamer, "-");
    if (verbose&0x1) {
        for (i=0;i<34;i++) streamer_printf(streamer, "-");
    }
    streamer_printf(streamer, "\n");


    inst->spi_bt_lock = 1;
    inst->sys_status_bt_lock = 1;
    while (spi_i<MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN) || irq_i<MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN)) {
        uint16_t spi_i_mod = (inst->spi_bt_idx + spi_i + 1) % MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN);
        uint16_t irq_i_mod = (inst->sys_status_bt_idx + irq_i + 1) % MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN);
        spi_p = &inst->spi_bt[spi_i_mod];
        irq_p = &inst->sys_status_bt[irq_i_mod];
        if ((spi_p->utime < irq_p->utime && spi_i < MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN)) ||
            irq_i >= MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN)) {
            if (print_spi_bt_line(&start_t, verbose, spi_p, spi_p_last, streamer)) {
                streamer_printf(streamer, "\n");
            }
            spi_p_last = spi_p;
            spi_i++;
        } else {
            streamer_printf(streamer, "\e[93m");   /* Set to bright yellow */
            if (print_interrupt_bt_line(&start_t, verbose, irq_p, irq_p_last, streamer)) {
                streamer_printf(streamer, "\e[39m\n");   /* Set to default colour */
            } else {
                streamer_printf(streamer, "\e[39m");     /* Set to default colour */
            }
            irq_p_last = irq_p;
            irq_i++;
        }
    }
    inst->spi_bt_lock = 0;
    inst->sys_status_bt_lock = 0;
}

#endif

#ifndef __KERNEL__
static void
dw1000_cli_too_few_args(struct streamer *streamer)
{
    streamer_printf(streamer, "Too few args\n");
}
#endif

static int
dw1000_cli_cmd(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *streamer)
{
#ifndef __KERNEL__
    struct _dw1000_dev_instance_t * inst = 0;
    uint16_t inst_n;

    if (argc < 2) {
        dw1000_cli_too_few_args(streamer);
        return 0;
    }

    if (!strcmp(argv[1], "dump")) {
        if (argc < 3) {
            inst_n=0;
        } else {
            inst_n = strtol(argv[2], NULL, 0);
        }
        inst = hal_dw1000_inst(inst_n);
        console_no_ticks();
        dw1000_cli_dump_registers(inst, streamer);
        console_yes_ticks();
#if MYNEWT_VAL(DW1000_CLI_EVENT_COUNTERS)
    } else if (!strcmp(argv[1], "ev")) {
        if (argc<4) {
            dw1000_cli_too_few_args(streamer);
            return 0;
        }
        inst_n = strtol(argv[2], NULL, 0);
        inst = hal_dw1000_inst(inst_n);
        if (!strcmp(argv[3], "on")) {
            dw1000_phy_event_cnt_ctrl(inst, true, true);
            streamer_printf(streamer, "ev on\n");
        } else if (!strcmp(argv[3], "reset")) {
            dw1000_phy_event_cnt_ctrl(inst, false, false);
            streamer_printf(streamer, "ev reset+off\n");
        } else if (!strcmp(argv[3], "dump")) {
            console_no_ticks();
            dw1000_cli_dump_event_counters(inst, streamer);
            console_yes_ticks();
        }
#endif
    } else if (!strcmp(argv[1], "da")) {
        if (argc<3) {
            return 0;
        }
        inst_n = strtol(argv[2], NULL, 0);
        uint32_t addr = strtol(argv[3], NULL, 0);
        int length = 128;
        if (argc>4) {
            length = strtol(argv[4], NULL, 0);
        }
        inst = hal_dw1000_inst(inst_n);
        dw1000_cli_dump_address(inst, addr, length, streamer);
    } else if (!strcmp(argv[1], "cw")) {
        if (argc<3) {
            inst_n=0;
        }
        inst_n = strtol(argv[2], NULL, 0);
        inst = hal_dw1000_inst(inst_n);
        hal_gpio_irq_disable(inst->irq_pin);
        dw1000_write_reg(inst, SYS_MASK_ID, 0, 0, sizeof(uint32_t));
        dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_TRXOFF, sizeof(uint8_t));
        dw1000_configcwmode(inst, inst->uwb_dev.config.channel);
        streamer_printf(streamer, "Device[%d] now in CW mode on ch %d. Reset to continue\n",
                        inst_n, inst->uwb_dev.config.channel);
    } else if (!strcmp(argv[1], "wr")) {
        if (argc < 7) {
            return 0;
        }
        inst_n = strtol(argv[2], NULL, 0);
        uint32_t addr = strtol(argv[3], NULL, 0);
        uint32_t sub  = strtol(argv[4], NULL, 0);
        uint64_t val  = strtol(argv[5], NULL, 0);
        int length = strtol(argv[6], NULL, 0);
        dw1000_write_reg(hal_dw1000_inst(inst_n), addr, sub, val, length);
    } else if (!strcmp(argv[1], "rd")) {
        if (argc < 6) {
            return 0;
        }
        inst_n = strtol(argv[2], NULL, 0);
        uint32_t addr = strtol(argv[3], NULL, 0);
        uint16_t sub  = strtol(argv[4], NULL, 0);
        int length = strtol(argv[5], NULL, 0);
        uint64_t reg = dw1000_read_reg(hal_dw1000_inst(inst_n), addr, sub, length);
        streamer_printf(streamer, "0x%06"PRIX32",0x%04X: 0x%"PRIX64"\n", addr, sub, reg);
#if MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN)
    } else if (!strcmp(argv[1], "ibt")){
        uint8_t d=0;
        if (argc < 3) {
            inst_n=0;
        } else {
            inst_n = strtol(argv[2], NULL, 0);
        }
        if (argc < 4) {
            d=0;
        } else {
            d = strtol(argv[3], NULL, 0);
        }
        inst = hal_dw1000_inst(inst_n);
        console_no_ticks();
        dw1000_cli_interrupt_backtrace(inst, d, streamer);
        console_yes_ticks();
    } else if (!strcmp(argv[1], "status2txt")){
        uint64_t d = strtoll(argv[2], NULL, 0);
        console_printf("%010llX: %s\n", d, sys_status_to_string(d));
    } else if (!strcmp(argv[1], "fctrl2txt")){
        uint8_t d=0,d2=0;
        if (argc < 4) {
            console_printf("2 bytes needed\n");
            return 0;
        } else {
            d = strtol(argv[2], NULL, 16);
            d2 = strtol(argv[3], NULL, 16);
        }
        streamer_printf(streamer, "%02X %02X: %s\n", (uint8_t)d, (uint8_t)d2, fctrl_to_string((d2<<8)|d));
        streamer_printf(streamer, "----\n ledgend: \n");
        fctrl_ledgend(streamer);
#endif
#if MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN)
    } else if (!strcmp(argv[1], "spibt")){
        uint8_t d=0;
        if (argc < 3) {
            inst_n=0;
        } else {
            inst_n = strtol(argv[2], NULL, 0);
        }
        if (argc < 4) {
            d=0;
        } else {
            d = strtol(argv[3], NULL, 0);
        }
        inst = hal_dw1000_inst(inst_n);
        console_no_ticks();
        dw1000_cli_spi_backtrace(inst, d, streamer);
        console_yes_ticks();
#endif
#if MYNEWT_VAL(DW1000_SYS_STATUS_BACKTRACE_LEN) && MYNEWT_VAL(DW1000_SPI_BACKTRACE_LEN)
    } else if (!strcmp(argv[1], "bt")){
        uint8_t d=0;
        if (argc < 3) {
            inst_n=0;
        } else {
            inst_n = strtol(argv[2], NULL, 0);
        }
        if (argc < 4) {
            d=0;
        } else {
            d = strtol(argv[3], NULL, 0);
        }
        inst = hal_dw1000_inst(inst_n);
        console_no_ticks();
        dw1000_cli_backtrace(inst, d, streamer);
        console_yes_ticks();
#endif
    } else {
        streamer_printf(streamer, "Unknown cmd\n");
    }
#endif  /* ifndef __KERNEL__ */

    return 0;
}

#endif


int
dw1000_cli_register(void)
{
#if MYNEWT_VAL(DW1000_CLI)
    int rc;
    rc = shell_cmd_register(&shell_dw1000_cmd);
#ifdef __KERNEL__
    {
        int i;
        struct _dw1000_dev_instance_t *inst;
        for (i=0;i<MYNEWT_VAL(UWB_DEVICE_MAX);i++) {
            inst = hal_dw1000_inst(i);
            if (!inst) continue;
            if (!inst->uwb_dev.status.initialized) continue;
            dw1000_sysfs_init(inst);
        }
        dw1000_debugfs_init();
    }
#endif
    return rc;
#else
    return 0;
#endif
}

int
dw1000_cli_down(int reason)
{
#ifdef __KERNEL__
    int i;
    struct _dw1000_dev_instance_t *inst;
    for (i=0;i<MYNEWT_VAL(UWB_DEVICE_MAX);i++) {
        inst = hal_dw1000_inst(i);
        if (!inst) continue;
        dw1000_sysfs_deinit(i);
    }
    dw1000_debugfs_deinit();
#endif
    return 0;
}
