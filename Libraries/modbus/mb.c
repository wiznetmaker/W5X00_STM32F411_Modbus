
#include "mbtcp.h"
#include "mbrtu.h"
#include "mbascii.h"

#include "socket.h"
#include "mb.h"
#include "modbus_serial.h"

extern volatile uint8_t* pucASCIIBufferCur;
extern volatile uint16_t usASCIIBufferPos;

void mbTCPtoRTU(uint8_t sock)
{
  if(MBtcp2rtuFrame(sock) != FALSE)
	{
		while(usRTUBufferPos)
		{
			UART_write((uint8_t*)pucRTUBufferCur, 1);
			pucRTUBufferCur++;
			usRTUBufferPos--;
		}
	}
}

void mbRTUtoTCP(uint8_t sock)
{
	if(MBrtu2tcpFrame() != FALSE)
      send(sock, (uint8_t*)pucTCPBufferCur, usTCPBufferPos);
}

void mbTCPtoASCII(uint8_t sock)
{
	uint8_t ucByte;
	
	if(MBtcp2asciiFrame(sock) != FALSE)
	{
		ucByte = MB_ASCII_START;
		UART_write(&ucByte, 1);
		while(usASCIIBufferPos)
		{
			ucByte = prvucMBBIN2CHAR((uint8_t)*pucASCIIBufferCur>>4);
			UART_write(&ucByte, 1);

			ucByte = prvucMBBIN2CHAR((uint8_t)*pucASCIIBufferCur&0x0F);
			UART_write(&ucByte, 1);

			pucASCIIBufferCur++;
			usASCIIBufferPos--;
		  }
		ucByte = MB_ASCII_DEFAULT_CR;
		UART_write(&ucByte, 1);
		ucByte = MB_ASCII_DEFAULT_LF;
		UART_write(&ucByte, 1);
	}
}

void mbASCIItoTCP(uint8_t sock)
{
	if(MBascii2tcpFrame() != FALSE)
    send(sock, (uint8_t*)pucTCPBufferCur, usTCPBufferPos);
}
