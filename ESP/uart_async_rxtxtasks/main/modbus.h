#ifndef _MODBUS_H_
#define _MODBUS_H_

#include "esp_system.h"
#include "driver/uart.h"

#define MODBUS_FUNC_READHOLDINGREGISTERS 3
#define MODBUS_FUNC_READINPUTREGISTERS 4
#define MODBUS_FUNC_WRITESINGLEREGISTER 6

#define MODBUS_STATES_UNDEF 0
#define MODBUS_STATES_RQSEND 1

#define MODBUS_HR_SIZE 64

#define MODBUS_WAIT_TIME 25

#define MODBUS_RESULT_OK 1
#define MODBUS_RESULT_NO_ANSWER 2
#define MODBUS_RESULT_BAD_CRC 3
#define MODBUS_RESULT_ERROR 4
#define MODBUS_RESULT_UNASKED 5
#define MODBUS_RESULT_WRONG_DATA 6

#define MODBUS_DEVICE_COUNT 3
#define MODBUS_STATUS_R 60


void MbSendGetHRSRQ(unsigned char deviceAddress, unsigned short dataAddress, unsigned short count, uart_port_t uart_num, unsigned short dataStartReg);
unsigned short MbCRCfunc(unsigned short inCRC, unsigned char in);

void MbSendSetRegister(unsigned char deviceAddress, unsigned short registerNum, short value, uart_port_t uart_num);

void MbParseAnswer(uint8_t* data, int rxBytes);

void MbSetResult(short result);

#endif