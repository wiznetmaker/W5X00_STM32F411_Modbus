
#include "modbus_serial.h"
#include "mbrtu.h"
#include "mbascii.h"

BUFFER_DEFINITION(uart_rx, UART_RX_SIZE);
extern uint8_t uartRxByte;
extern UART_HandleTypeDef huart1;

/*****************************************************************************
 * Private functions
 ****************************************************************************/
/**
 * @brief	UART interrupt handler using ring buffers
 * @return	Nothing
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) // USART1
    {
        if(is_uart_buffer_full() == FALSE)
            put_byte_to_uart_buffer(uartRxByte);
        HAL_UART_Receive_IT(&huart1, &uartRxByte, 1);
    }
}

int UART_read(void *data, int bytes)
{
  uint32_t i;
  uint8_t *data_ptr = data;
  if(IS_BUFFER_EMPTY(uart_rx)) return RET_NOK;
  
  for(i=0; i<bytes; i++)
    data_ptr[i] = (uint8_t)BUFFER_OUT(uart_rx);
    //*((uint8_t *)(data + i)) = (uint8_t)BUFFER_OUT(uart_rx);
  BUFFER_OUT_MOVE(uart_rx, i);
  return i;
	//return Chip_UART_ReadRB(LPC_USART, &rxring, data, bytes);  
}

uint32_t UART_write(void *data, int bytes)
{
  HAL_UART_Transmit(&huart1, data, bytes, 1000);
  return bytes;
//	return Chip_UART_SendRB(LPC_USART, &txring, data, bytes);
}

int32_t platform_uart_getc(void)
{
    int32_t ch;

    while(IS_BUFFER_EMPTY(uart_rx));
    ch = (int32_t)BUFFER_OUT(uart_rx);
    BUFFER_OUT_MOVE(uart_rx, 1);

    return ch;
}

int32_t platform_uart_getc_nonblk(void)
{
    int32_t ch;

    if(IS_BUFFER_EMPTY(uart_rx)) return RET_NOK;
    ch = (int32_t)BUFFER_OUT(uart_rx);
    BUFFER_OUT_MOVE(uart_rx, 1);

    return ch;
}

int32_t platform_uart_gets(uint8_t* buf, uint16_t bytes)
{
    uint16_t lentot = 0, len1st = 0;

    lentot = bytes = MIN(BUFFER_USED_SIZE(uart_rx), bytes);
    if(IS_BUFFER_OUT_SEPARATED(uart_rx) && (len1st = BUFFER_OUT_1ST_SIZE(uart_rx)) < bytes) {
        memcpy(buf, &BUFFER_OUT(uart_rx), len1st);
        BUFFER_OUT_MOVE(uart_rx, len1st);
        bytes -= len1st;
    }
    memcpy(buf+len1st, &BUFFER_OUT(uart_rx), bytes);
    BUFFER_OUT_MOVE(uart_rx, bytes);

    return lentot;
}

void uart_rx_flush(void)
{
    BUFFER_CLEAR(uart_rx);
}

void put_byte_to_uart_buffer(uint8_t ch)
{
    BUFFER_IN(uart_rx) = ch;
    BUFFER_IN_MOVE(uart_rx, 1);
}


uint16_t get_uart_buffer_usedsize(void)
{
    uint16_t len = 0;

    len = BUFFER_USED_SIZE(uart_rx);
    return len;
}

uint16_t get_uart_buffer_freesize(void)
{
    uint16_t len = 0;

    len = BUFFER_FREE_SIZE(uart_rx);
    return len;
}

int8_t is_uart_buffer_empty(void)
{
    int8_t ret = 0;

    ret = IS_BUFFER_EMPTY(uart_rx);
    return ret;
}

int8_t is_uart_buffer_full(void)
{
    int8_t ret = 0;

    ret = IS_BUFFER_FULL(uart_rx);
    return ret;
}

