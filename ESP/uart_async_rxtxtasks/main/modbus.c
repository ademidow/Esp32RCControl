
#include "esp_system.h"
#include "esp_log.h"
#include "modbus.h"
#include "driver/uart.h"

unsigned char rw_buffer[256];
short mb_data[MODBUS_HR_SIZE];
unsigned short state;
unsigned short regFrom;

void MbSendGetHRSRQ(unsigned char deviceAddress, unsigned short dataAddress, unsigned short count, uart_port_t uart_num, unsigned short dataStartReg)
{
    state = MODBUS_STATES_RQSEND;
    regFrom = dataStartReg;
    unsigned short  crc = 0xFFFF;
    rw_buffer[0] = deviceAddress;
    rw_buffer[1] = MODBUS_FUNC_READHOLDINGREGISTERS;
    rw_buffer[2] = (unsigned char)((dataAddress>>8)&0xFF);
    //rw_buffer[2] = 0;
    rw_buffer[3] = (unsigned char)(dataAddress&0xFF);
    rw_buffer[4] = (unsigned char)((count>>8)&0xFF);
    rw_buffer[5] = (unsigned char)(count&0xFF);
    for(int i = 0; i<6; i++)
        crc = MbCRCfunc(crc, rw_buffer[i]);
    rw_buffer[6] = (unsigned char)(crc&0xFF);
    rw_buffer[7] = (unsigned char)((crc>>8)&0xFF);
    uart_write_bytes(uart_num, rw_buffer, 8);
}

void MbSendSetRegister(unsigned char deviceAddress, unsigned short registerNum, short value, uart_port_t uart_num)
{
    state = MODBUS_STATES_RQSEND;
    unsigned short  crc = 0xFFFF;
    rw_buffer[0] = deviceAddress;
    rw_buffer[1] = MODBUS_FUNC_WRITESINGLEREGISTER;
    rw_buffer[2] = (unsigned char)((registerNum>>8)&0xFF);
    rw_buffer[3] = (unsigned char)(registerNum&0xFF);
    rw_buffer[4] = (unsigned char)((value>>8)&0xFF);
    rw_buffer[5] = (unsigned char)(value&0xFF);
    for(int i = 0; i<6; i++)
        crc = MbCRCfunc(crc, rw_buffer[i]);
    rw_buffer[6] = (unsigned char)(crc&0xFF);
    rw_buffer[7] = (unsigned char)((crc>>8)&0xFF);
    uart_write_bytes(uart_num, rw_buffer, 8);
}

void MbParseAnswer(uint8_t* data, int rxBytes)
{
    //static const char *RX_TASK_TAG = "RX_TASK";
    if (state == MODBUS_STATES_RQSEND && rxBytes == (rw_buffer[5]<<1) + 5)
        {
            if (data[0] == rw_buffer[0] && data[1] == rw_buffer[1])
            {
                unsigned short  crc = 0xFFFF;
                if (data[1] == MODBUS_FUNC_READHOLDINGREGISTERS)
                {
                    for(int i = 0; i < rxBytes-2 ; i++)
                        crc = MbCRCfunc(crc, data[i]);
                    //ESP_LOGI(RX_TASK_TAG, "Read %d bytes from device %d, CRC in data %d, CRC calculated %d", rxBytes, data[0], (data[rxBytes-1]<<8)+data[rxBytes-2], crc);
                    if (crc == (unsigned short)((data[rxBytes-1]<<8)+data[rxBytes-2]))
                    {
                        for (int j = 0; j < rw_buffer[5]; j++)
                        {
                            mb_data[j + regFrom] = (short)(data[j*2+4] + (data[j*2+3]<<8));
                            //ESP_LOGI(RX_TASK_TAG, "Register %d - %d ", j, mb_data[j + regFrom]);
                        }
                        MbSetResult(MODBUS_RESULT_OK);
                    }
                    else
                    {
                        //ESP_LOGI(RX_TASK_TAG, "Read %d bytes, bad CRC!!! CRC in data %d, CRC calculated %d", rxBytes, (data[rxBytes-1]<<8)+data[rxBytes-2], crc);
                        MbSetResult(MODBUS_RESULT_BAD_CRC);
                    }
                }
                if (data[1] == MODBUS_FUNC_WRITESINGLEREGISTER)
                {
                    //MbSetResult(MODBUS_RESULT_OK);
                }
                state = MODBUS_STATES_UNDEF;
            }
            else
            {
                //ESP_LOGI(RX_TASK_TAG, "Read %d bytes, not valid query", rxBytes);
                MbSetResult(MODBUS_RESULT_WRONG_DATA);
            }
        }
        else
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
            if (rxBytes > 0) 
            {
                data[rxBytes] = 0;
                //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
                //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
                MbSetResult(MODBUS_RESULT_UNASKED);

            }
            else 
            {
                
                //ESP_LOGI(RX_TASK_TAG, "No answer from device %d", rw_buffer[0]);
                MbSetResult(MODBUS_RESULT_NO_ANSWER);
            }
        }
    state = MODBUS_STATES_UNDEF;

}

unsigned short MbCRCfunc(unsigned short inCRC, unsigned char in)
{
   unsigned short j = 0;
   inCRC=inCRC^in;
   for(j=0;j<8;j++)
   {
        if(inCRC&1)
        {
            inCRC=(inCRC>>1)^0xA001U;
        }
        else 
        {
            inCRC=inCRC>>1;
        }
    }
   return inCRC;
}

void MbSetResult(short result)
{
    if (rw_buffer[0] <= MODBUS_DEVICE_COUNT)
    {
        mb_data[MODBUS_STATUS_R + rw_buffer[0] -1] = result;
    }
}