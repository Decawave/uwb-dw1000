#ifndef _DW1000_CLI_PRIV_H_
#define _DW1000_CLI_PRIV_H_

typedef int (*cli_out_cb_t)(const char *fmt, ...);
void dw1000_cli_dump_registers(struct _dw1000_dev_instance_t * inst, cli_out_cb_t cb);
void dw1000_cli_dump_event_counters(struct _dw1000_dev_instance_t * inst, cli_out_cb_t cb);
void dw1000_cli_backtrace(struct _dw1000_dev_instance_t * inst, uint16_t verbose, cli_out_cb_t cb);
void dw1000_cli_spi_backtrace(struct _dw1000_dev_instance_t * inst, uint16_t verbose, cli_out_cb_t cb);
void dw1000_cli_interrupt_backtrace(struct _dw1000_dev_instance_t * inst, uint16_t verbose, cli_out_cb_t cb);

#endif /* _DW1000_CLI_PRIV_H_ */
