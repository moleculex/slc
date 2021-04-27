#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "stpm32.h"
#include "nrf_uart.h"
#include "app_uart.h"
#include "boards.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_error.h"
#include "protocol.h"

void stpm32_frame(uint8_t *pBuf);
uint8_t CalcCRC8(uint8_t *pBuf);
void Crc8Calc (uint8_t u8Data);
uint8_t stpm32_byteReverse(uint8_t n);

void stpm32_uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

void stpm32_init(void)
{
  ret_code_t err_code;

  memset(_stpm32.rxBuffer, '\0', sizeof _stpm32.rxBuffer);
  _stpm32.rxBuffer_ptr = &_stpm32.rxBuffer[0];

  const app_uart_comm_params_t comm_params =
  {
      29,
      45,
      0,
      0,
      0,
      false,
      NRF_UART_BAUDRATE_115200
  };

  APP_UART_FIFO_INIT(&comm_params,
                     256,
                     256,
                     stpm32_uart_error_handle,
                     APP_IRQ_PRIORITY_LOWEST,
                     err_code);

  APP_ERROR_CHECK(err_code);

#ifdef V1
	//RST
	GPIO_InitStructure.GPIO_Pin = STPM32_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(STPM32_PORT, &GPIO_InitStructure);
	GPIO_SetBits(STPM32_PORT, STPM32_TX_PIN);
	vTaskDelay(10);
	GPIO_ResetBits(STPM32_PORT, STPM32_TX_PIN);
	vTaskDelay(10);
	GPIO_SetBits(STPM32_PORT, STPM32_TX_PIN);
	vTaskDelay(10);
#endif
}

int vrms, irms, pha;
char tmp[2];

typedef struct {
	union {
		char buffer[4];
		struct {
			int vrms : 15;
			int irms : 17;
		}__attribute__((packed));
	};
}t_vi;

t_vi _vi;

typedef struct {
	union {
		char buffer[4];
		struct {
			int reserved1 : 16;
			int pha : 12;
			int reserved2 : 4;
		}__attribute__((packed));
	};
}t_pha;

t_pha _pha;

int v;

void stpm32_manager(void)
{
	_stpm32.state = 0;

	for(;;)
	{
		switch(_stpm32.state)
		{
			case 0:
				stpm32_init();
				vTaskDelay(2000);
				_stpm32.state = 1;

				break;

			case 1:
				memset(_stpm32.rxBuffer, '\0', 20);
				_stpm32.rxBuffer_ptr = &_stpm32.rxBuffer[0];
				stpm32_enableLatch();
				vTaskDelay(1000);

				memset(_stpm32.rxBuffer, '\0', 20);
				_stpm32.rxBuffer_ptr = &_stpm32.rxBuffer[0];
				stpm32_readVI();
				vTaskDelay(500);

				_vi.buffer[0] = _stpm32.rxBuffer[5];
				_vi.buffer[1] = _stpm32.rxBuffer[6];
				_vi.buffer[2] = _stpm32.rxBuffer[7];
				_vi.buffer[3] = _stpm32.rxBuffer[8];

				_status.voltage[1] = _vi.vrms & 0xff;
				_status.voltage[0] = (_vi.vrms >> 8) & 0xff;
				_status.current[1] = _vi.irms & 0xff;
				_status.current[0] = (_vi.irms >> 8) & 0xff;

				//_vi.vrms = 0x19EC;
				v = 0.98 *_vi.vrms * 1.2 * (1 + 810000 / 470) / (0.875 * 2 * pow(2, 15));

				if(v < 200)
					_sys.led = 0;
				else
					_sys.led = 1;

				memset(_stpm32.rxBuffer, '\0', 20);
				_stpm32.rxBuffer_ptr = &_stpm32.rxBuffer[0];
				stpm32_readPHA();
				vTaskDelay(500);

				_pha.buffer[0] = _stpm32.rxBuffer[5];
				_pha.buffer[1] = _stpm32.rxBuffer[6];
				_pha.buffer[2] = _stpm32.rxBuffer[7];
				_pha.buffer[3] = _stpm32.rxBuffer[8];

				_status.phase_angle[1] = _pha.pha & 0xff;
				_status.phase_angle[0] = (_pha.pha >> 8) & 0xff;

				vTaskDelay(1000);

				break;
		}
	}
}

void stpm32_enableLatch(void)
{
        app_uart_flush();
        _stpm32.rxBuffer_ptr = &_stpm32.rxBuffer[0];

	_stpm32.txBuffer[0] = 0x04;
	_stpm32.txBuffer[1] = 0x05;
	_stpm32.txBuffer[2] = 0x80;
	_stpm32.txBuffer[3] = 0x00;
	stpm32_frame((uint8_t *)_stpm32.txBuffer);
	stpm32_write(_stpm32.txBuffer, 5);

	_stpm32.txBuffer[0] = 0xff;
	_stpm32.txBuffer[1] = 0xff;
	_stpm32.txBuffer[2] = 0xff;
	_stpm32.txBuffer[3] = 0xff;
	stpm32_frame((uint8_t *)_stpm32.txBuffer);
	stpm32_write(_stpm32.txBuffer, 5);

        stpm32_read(_stpm32.rxBuffer, 50);
}

void stpm32_readVI(void)
{
        app_uart_flush();
        _stpm32.rxBuffer_ptr = &_stpm32.rxBuffer[0];

	_stpm32.txBuffer[0] = 0x48;
	_stpm32.txBuffer[1] = 0xff;
	_stpm32.txBuffer[2] = 0xff;
	_stpm32.txBuffer[3] = 0xff;
	stpm32_frame((uint8_t *)_stpm32.txBuffer);
	stpm32_write(_stpm32.txBuffer, 5);

	_stpm32.txBuffer[0] = 0xff;
	_stpm32.txBuffer[1] = 0xff;
	_stpm32.txBuffer[2] = 0xff;
	_stpm32.txBuffer[3] = 0xff;
	stpm32_frame((uint8_t *)_stpm32.txBuffer);
	stpm32_write(_stpm32.txBuffer, 5);

        stpm32_read(_stpm32.rxBuffer, 50);
}

void stpm32_readPHA(void)
{
        app_uart_flush();
        _stpm32.rxBuffer_ptr = &_stpm32.rxBuffer[0];

	_stpm32.txBuffer[0] = 0x4E;
	_stpm32.txBuffer[1] = 0xff;
	_stpm32.txBuffer[2] = 0xff;
	_stpm32.txBuffer[3] = 0xff;
	stpm32_frame((uint8_t *)_stpm32.txBuffer);
	stpm32_write(_stpm32.txBuffer, 5);

	_stpm32.txBuffer[0] = 0xff;
	_stpm32.txBuffer[1] = 0xff;
	_stpm32.txBuffer[2] = 0xff;
	_stpm32.txBuffer[3] = 0xff;
	stpm32_frame((uint8_t *)_stpm32.txBuffer);
	stpm32_write(_stpm32.txBuffer, 5);

        stpm32_read(_stpm32.rxBuffer, 50);
}

void stpm32_frame(uint8_t *pBuf)
{
	 uint8_t temp[4], x, CRC_on_reversed_buf;

	 for(x=0; x < 4; x++)
	 {
		 temp[x] = stpm32_byteReverse(pBuf[x]);
	 }

	 CRC_on_reversed_buf = CalcCRC8(temp);
	 pBuf[4] = stpm32_byteReverse(CRC_on_reversed_buf);
}

uint8_t stpm32_byteReverse(uint8_t n)
{
	 n = ((n >> 1) & 0x55) | ((n << 1) & 0xaa);
	 n = ((n >> 2) & 0x33) | ((n << 2) & 0xcc);
	 n = ((n >> 4) & 0x0F) | ((n << 4) & 0xF0);

	 return n;
}

static uint8_t CRC_u8Checksum;

uint8_t CalcCRC8(uint8_t *pBuf)
{
	 uint8_t i;
	 CRC_u8Checksum = 0x00;
	 for (i = 0 ; i < 4; i++)
	 {
		 Crc8Calc(pBuf[i]);
	 }

	 return CRC_u8Checksum;
}

void Crc8Calc (uint8_t u8Data)
{
	 uint8_t loc_u8Idx = 0;
	 uint8_t loc_u8Temp;
	 uint8_t CRC_8 = 0x07;

	 while(loc_u8Idx < 8)
	 {
		 loc_u8Temp = u8Data ^ CRC_u8Checksum;
		 CRC_u8Checksum <<= 1;

		 if(loc_u8Temp & 0x80)
		 {
			 CRC_u8Checksum ^= CRC_8;
		 }

		 u8Data<<=1;
		 loc_u8Idx++;
	 }
}

void stpm32_write(char *buffer, char length)
{
  for(int n = 0; n < length; n++)
    while (app_uart_put(buffer[n]) != NRF_SUCCESS);
}

void stpm32_read(uint8_t *buffer, int length)
{
  uint8_t c;

  for(int n = 0; n < length; n++)
  {
      app_uart_get(&c);
      buffer[n] = c;
  }
}