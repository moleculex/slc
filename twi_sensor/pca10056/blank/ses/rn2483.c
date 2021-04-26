#include <string.h>
#include <stdlib.h>
#include "rn2483.h"
#include "nrf_uart.h"
#include "app_uart.h"
#include "boards.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_error.h"
#include "protocol.h"

#define SYS_RESET "sys reset\r\n"
#define SYS_GET_HWEUI "sys get hweui\r\n"
#define SYS_SLEEP "sys sleep 4294967295\r\n"

#define MAC_SET_APPKEY "mac set appkey 13341334133413341334133413341334\r\n"
#define MAC_SET_APPEUI "mac set appeui 70B3D576AE000007\r\n"
#define MAC_SET_DR "mac set dr 1\r\n"
#define MAC_SET_ADR "mac set adr off\r\n"
#define MAC_JOIN_OTAA "mac join otaa\r\n"

#define MAC_RESUME "mac resume\r\n"
#define MAC_PAUSE "mac pause\r\n"

#define RADIO_SET_SF "radio set sf sf12\r\n"
#define RADIO_SET_RXBW "radio set rxbw 125\r\n"
#define RADIO_SET_FREQ "radio set freq 869525000\r\n"
//#define RADIO_SET_WDT "radio set wdt 60000\r\n"
#define RADIO_RX "radio rx 0\r\n"

int timer = 0;
int gpsTimer = 0;
char alarm = 0x00;

char ascii[64];
char ts[1];
unsigned int r;
unsigned int timeslot;
char strTimeslot[5];
char txBuf[21];
int f;

void rn2483_uart_error_handle(app_uart_evt_t * p_event)
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

void rn2483_init(void)
{
  ret_code_t err_code;

  memset(_rn2483.rxBuffer, '\0', sizeof _rn2483.rxBuffer);
  _rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];

  nrf_gpio_cfg(RN2483_RESET,
               NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0D1,
               NRF_GPIO_PIN_NOSENSE);

  const app_uart_comm_params_t comm_params =
  {
      RX_PIN_NUMBER,
      TX_PIN_NUMBER,
      0,
      0,
      0,
      false,
      NRF_UART_BAUDRATE_57600
  };

  APP_UART_FIFO_INIT(&comm_params,
                     256,
                     256,
                     rn2483_uart_error_handle,
                     APP_IRQ_PRIORITY_LOWEST,
                     err_code);

  APP_ERROR_CHECK(err_code);
}

void rn2483_task(void)
{
	char *index;

	_sys.status = 0;
	_rn2483.session_timer = 0;

	rn2483_init();
	rn2483_reset();
	//randomise frequency and assign timeslot
	hexStr_to_hex(ts, &_rn2483.deveui[14], 2);

	r = ts[0] /*^ _kx122.tilt.zout_l*/ * 20000;
	r /= 255;

	srand1(r);

	timeslot = rand1() % 20000;
	vTaskDelay(timeslot);
	itoa((60000 + timeslot), strTimeslot, 10);

	rn2483_rfInit();

	ascii[0] = 'C';
	ascii[1] = '0';
	memcpy(&ascii[2], &_rn2483.deveui[8], 8);
	_status.start = 0x80;
	_status.end = 0xfe;
	hex_to_hexStr(_status.buffer, &ascii[10], 14);
	rn2483_tx(ascii, 38);
	vTaskDelay(2000);

	nrf_gpio_pin_write(BUZZER, 1);
	vTaskDelay(200);
	nrf_gpio_pin_write(BUZZER, 0);
	rn2483_rx();

	for(;;)
	{
                rn2483_read(_rn2483.rxBuffer, 64);
		//radio_rx  00CB1AEB007F00000100FE
		index = strstr(_rn2483.rxBuffer, "radio_rx");
		if(index)
		{
			vTaskDelay(200);

			hexStr_to_hex(_protocol.buffer, &index[12], 12);

			if(strncmp(&index[12], _rn2483.devaddr, 8) == 0 /*|| (_protocol.profile == _flash.profile && _protocol.group == _flash.group)*/)
			{
				protocol_decode(&_protocol);
				nrf_gpio_pin_write(BUZZER, 1);
				vTaskDelay(200);
				nrf_gpio_pin_write(BUZZER, 0);
			}

			rn2483_rx();
		}

		index = strstr(_rn2483.rxBuffer, "radio_err");
		if(index)
		{
			vTaskDelay(200);
			if(timer == 10 || _sys.status == 1)
			{
				rn2483_rfInit();


				ascii[0] = 'C';
				ascii[1] = '0';
				memcpy(&ascii[2], &_rn2483.deveui[8], 8);
				_status.start = 0x80;
				_status.end = 0xfe;
				hex_to_hexStr(_status.buffer, &ascii[10], 14);
				rn2483_tx(ascii, 38);
				vTaskDelay(2000);

				nrf_gpio_pin_write(BUZZER, 1);
				vTaskDelay(200);
				nrf_gpio_pin_write(BUZZER, 0);
				timer = 0;
				alarm = 0x00;
				_sys.status = 0;
				//_stpm32.state = 0;
			}
			else
			{
				if((_status.tilt == 0x01 || _status.seismic == 0x01) && alarm == 0x00)
				{
					rn2483_rfInit();
					ascii[0] = 'C';
					ascii[1] = '0';
					memcpy(&ascii[2], &_rn2483.deveui[8], 8);
					_status.start = 0x7f;
					_status.end = 0xfe;
					hex_to_hexStr(_status.buffer, &ascii[10], 12);
					rn2483_tx(ascii, 34);
					vTaskDelay(2000);
					nrf_gpio_pin_write(BUZZER, 1);
					vTaskDelay(200);
					nrf_gpio_pin_write(BUZZER, 0);
					alarm = 0x01;
					_status.seismic = 0x00;
					_status.tilt = 0x00;
				}
				else
				{
					if(_sys.fix == 0x01 && _sys.sent == 0x00 && gpsTimer >= 4)
					{
						_sys.sent = 0x01;
						gpsTimer = 4;
						rn2483_rfInit();
						ascii[0] = 'C';
						ascii[1] = '0';
						memcpy(&ascii[2], &_rn2483.deveui[8], 8);
						ascii[10] = '8';
						ascii[11] = 'F';
						ascii[12] = '0';
						memcpy(&ascii[13], _sys.latitude, 10);
						memcpy(&ascii[23], _sys.longitude, 11);
						rn2483_tx(ascii, 34);
						vTaskDelay(2000);
						nrf_gpio_pin_write(BUZZER, 1);
						vTaskDelay(200);
						nrf_gpio_pin_write(BUZZER, 0);
					}
				}
			}

			timer++;
			gpsTimer++;

			rn2483_rx();
		}

		if(_rn2483.session_timer > 60 * 60 * 6)
		{
			_sys.fix = 0x00;
			_sys.sent = 0x00;
			gpsTimer = 0;
			_rn2483.session_timer = 0;
		}

		_rn2483.session_timer++;
		vTaskDelay(1000);
	}
}

void rn2483_reset(void)
{
        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);

	nrf_gpio_pin_write(RN2483_RESET, 1);
	vTaskDelay(1000);
	nrf_gpio_pin_write(RN2483_RESET, 0);
	vTaskDelay(1000);
	nrf_gpio_pin_write(RN2483_RESET, 1);
	vTaskDelay(1000);

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	rn2483_write(SYS_RESET, 11);
	vTaskDelay(500);

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	rn2483_write(SYS_GET_HWEUI, 15);
	vTaskDelay(200);

        rn2483_read(_rn2483.rxBuffer, 50);
	memcpy(&_rn2483.deveui, &_rn2483.rxBuffer, 16);
	memcpy(&_rn2483.devaddr, &_rn2483.rxBuffer[8], 8);
}

void rn2483_rfInit(void)
{
        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	rn2483_write(MAC_PAUSE, 11);
	vTaskDelay(200);

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	rn2483_write("radio set mod lora\r\n", 20);
	vTaskDelay(200);

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	rn2483_write("radio set pwr 14\r\n", 18);
	vTaskDelay(200);

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	rn2483_write("radio set sf sf11\r\n", 19);
	vTaskDelay(200);

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	rn2483_write(RADIO_SET_RXBW, 20);
	vTaskDelay(200);

	f = rand1() % 3;

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	switch(f)
	{
		/*case 0:
			rn2483_write("radio set freq 867100000\r\n", 26);
			break;

		case 1:
			rn2483_write("radio set freq 867300000\r\n", 26);
			break;

		case 2:
			rn2483_write("radio set freq 867500000\r\n", 26);
			break;

		case 3:
			rn2483_write("radio set freq 867700000\r\n", 26);
			break;

		case 4:
			rn2483_write("radio set freq 867900000\r\n", 26);
			break;*/

		case 0:
			rn2483_write("radio set freq 868100000\r\n", 26);
			break;

		case 1:
			rn2483_write("radio set freq 868300000\r\n", 26);
			break;

		case 2:
			rn2483_write("radio set freq 868500000\r\n", 26);
			break;

	}
	vTaskDelay(200);
}

void rn2483_rx(void)
{
        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	rn2483_write(MAC_PAUSE, 11);
	vTaskDelay(200);

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	rn2483_write(RADIO_SET_SF, 19);
	vTaskDelay(200);

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	rn2483_write(RADIO_SET_RXBW, 20);
	vTaskDelay(200);

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	rn2483_write(RADIO_SET_FREQ, 26);
	vTaskDelay(200);

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	rn2483_write("radio set iqi on\r\n", 18);
	vTaskDelay(200);

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	strcpy(txBuf, "radio set wdt ");
	strcat(txBuf, strTimeslot);
	strcat(txBuf, "\r\n");
	rn2483_write(txBuf, strlen(txBuf));
	vTaskDelay(200);

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
	rn2483_write(RADIO_RX, 12);
	vTaskDelay(200);

        app_uart_flush();
	_rn2483.rxBuffer_ptr = &_rn2483.rxBuffer[0];
	memset(&_rn2483.rxBuffer[0], '\0', sizeof _rn2483.rxBuffer);
}

void rn2483_tx(char *buffer, char length)
{
	memcpy(&_rn2483.txBuffer, (char *)"radio tx ", 9);
	memcpy(&_rn2483.txBuffer[9], buffer, length);
	memcpy(&_rn2483.txBuffer[9 + length], (char *)"\r\n", 2);
	rn2483_write(_rn2483.txBuffer,(9 + length + 2));
}

void rn2483_read(uint8_t *buffer, int length)
{
  uint8_t c;

  for(int n = 0; n < length; n++)
  {
    //while (app_uart_get(&c) != NRF_SUCCESS);
      app_uart_get(&c);
      buffer[n] = c;
  }
}

void rn2483_write(char *buffer, char length)
{
  for(int n = 0; n < length; n++)
    while (app_uart_put(buffer[n]) != NRF_SUCCESS);
}

void hexStr_to_hex(char *result, char *data, int byte_count)
{
    for(int c = 0; c < byte_count; c++)
    {
        if((*data & 0x30) == 0x30)
        {
            *result = (*data & 0x0F) << 4;
        }
        else
        {
            switch(*data)
            {
                case 'A':
                    *result = 0xA0;
                    break;

                case 'B':
                    *result = 0xB0;
                    break;

                case 'C':
                    *result = 0xC0;
                    break;

                case 'D':
                    *result = 0xD0;
                    break;

                case 'E':
                    *result = 0xE0;
                    break;

                case 'F':
                    *result = 0xF0;
                    break;
            }
        }
        *data++;
        if((*data & 0x30) == 0x30)
        {
            *result |= (*data & 0x0F);
        }
        else
        {
            switch(*data)
            {
                case 'A':
                    *result |= 0x0A;
                    break;

                case 'B':
                    *result |= 0x0B;
                    break;

                case 'C':
                    *result |= 0x0C;
                    break;

                case 'D':
                    *result |= 0x0D;
                    break;

                case 'E':
                    *result |= 0x0E;
                    break;

                case 'F':
                    *result |= 0x0F;
                    break;
            }
        }

        *result++;
        *data++;
    }
}

void hex_to_hexStr(char *hex, char *ascii, int byte_count)
{
    unsigned char a;
    for(int i = 0; i < byte_count; i++)
    {
        a=*hex;
        a>>=4;
        a&=0x0f;
        if(a<10)
        {
            a|=0x30;
            *ascii=a;
        }
        else
        {
            switch(a)
            {
                case 10:
                    *ascii='A';
                    break;
                case 11:
                    *ascii='B';
                    break;
                case 12:
                    *ascii='C';
                    break;
                case 13:
                    *ascii='D';
                    break;
                case 14:
                    *ascii='E';
                    break;
                case 15:
                    *ascii='F';
                    break;
            }
        }
        *ascii++;

        a=*hex;
        a&=0x0f;
        if(a<10)
        {
            a|=0x30;
            *ascii=a;
        }
        else
        {
            switch(a)
            {
                case 10:
                    *ascii='A';
                    break;
                case 11:
                    *ascii='B';
                    break;
                case 12:
                    *ascii='C';
                    break;
                case 13:
                    *ascii='D';
                    break;
                case 14:
                    *ascii='E';
                    break;
                case 15:
                    *ascii='F';
                    break;
            }
        }
        *ascii++;
        *hex++;
    }
}

static uint32_t miState;

int32_t rand1( void )
{
    miState ^= (miState << 13);
    miState ^= (miState >> 17);
    miState ^= (miState << 15);

    return (miState * 1332534557) & 0x7FFFFFFF;
}

void srand1( uint32_t seed )
{
    // a zero seed will not work!
    if (seed == 0)
        seed = 0x55aaff00;

    miState = seed;
}