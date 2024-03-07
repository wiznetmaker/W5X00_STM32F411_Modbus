
#ifndef __MODBUS_SERIAL_H__
#define __MODBUS_SERIAL_H__

#include "port_common.h"

/* Transmit and receive ring buffer sizes */
#define UART_TX_SIZE 1024	/* Send */
#define UART_RX_SIZE 1024	/* Receive */

#define RET_OK                  0
#define RET_NOK                 -1
#define RET_TIMEOUT             -2

#define BUFFER_DEFINITION(_name, _size) \
    uint8_t _name##_buf[_size]; \
    volatile uint16_t _name##_wr=0; \
    volatile uint16_t _name##_rd=0; \
    volatile uint16_t _name##_sz=_size;
#define BUFFER_DECLARATION(_name) \
    extern uint8_t _name##_buf[]; \
    extern uint16_t _name##_wr, _name##_rd, _name##_sz;
#define BUFFER_CLEAR(_name) \
    _name##_wr=0;\
    _name##_rd=0;

#define BUFFER_USED_SIZE(_name) ((_name##_sz + _name##_wr - _name##_rd) % _name##_sz)
#define BUFFER_FREE_SIZE(_name) ((_name##_sz + _name##_rd - _name##_wr - 1) % _name##_sz)
#define IS_BUFFER_EMPTY(_name) ( (_name##_rd) == (_name##_wr))
#define IS_BUFFER_FULL(_name) (BUFFER_FREE_SIZE(_name) == 0)	// I guess % calc takes time a lot, so...
//#define IS_BUFFER_FULL(_name) ((_name##_rd!=0 && _name##_wr==_name##_rd-1)||(_name##_rd==0 && _name##_wr==_name##_sz-1))

#define BUFFER_IN(_name) _name##_buf[_name##_wr]
#define BUFFER_IN_OFFSET(_name, _offset) _name##_buf[_name##_wr + _offset]
#define BUFFER_IN_MOVE(_name, _num) _name##_wr = (_name##_wr + _num) % _name##_sz
#define BUFFER_IN_1ST_SIZE(_name) (_name##_sz - _name##_wr - ((_name##_rd==0)?1:0))
#define BUFFER_IN_2ND_SIZE(_name) ((_name##_rd==0) ? 0 : _name##_rd-1)
#define IS_BUFFER_IN_SEPARATED(_name) (_name##_rd <= _name##_wr)

#define BUFFER_OUT(_name) _name##_buf[_name##_rd]
#define BUFFER_OUT_OFFSET(_name, _offset) _name##_buf[_name##_rd + _offset]
#define BUFFER_OUT_MOVE(_name, _num) _name##_rd = (_name##_rd + _num) % _name##_sz
#define BUFFER_OUT_1ST_SIZE(_name) (_name##_sz - _name##_rd)
#define BUFFER_OUT_2ND_SIZE(_name) (_name##_wr)
#define IS_BUFFER_OUT_SEPARATED(_name) (_name##_rd > _name##_wr)

void uart_rx_flush(void);
void put_byte_to_uart_buffer(uint8_t ch);
uint16_t get_uart_buffer_usedsize(void);
uint16_t get_uart_buffer_freesize(void);
int8_t is_uart_buffer_empty(void);
int8_t is_uart_buffer_full(void);

int UART_read(void *data, int bytes);
uint32_t UART_write(void *data, int bytes);

#endif
