#include <string.h>
#include <stdlib.h>
#include "sara.h"
#include "nrf_uart.h"
#include "app_uart.h"
#include "boards.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_error.h"
#include "protocol.h"

void sara_uart_error_handle(app_uart_evt_t * p_event)
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

void sara_init(void)
{
  ret_code_t err_code;

  memset(_sara.rxBuffer, '\0', sizeof _sara.rxBuffer);
  _sara.rxBuffer_ptr = &_sara.rxBuffer[0];

  nrf_gpio_cfg(SARA_PWR,
               NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0D1,
               NRF_GPIO_PIN_NOSENSE);

  nrf_gpio_cfg(SARA_RST,
               NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0D1,
               NRF_GPIO_PIN_NOSENSE);

  const app_uart_comm_params_t comm_params =
  {
      17,
      29,
      0,
      0,
      0,
      false,
      NRF_UART_BAUDRATE_115200
  };

  APP_UART_FIFO_INIT(&comm_params,
                     256,
                     256,
                     sara_uart_error_handle,
                     APP_IRQ_PRIORITY_LOWEST,
                     err_code);

  APP_ERROR_CHECK(err_code);
}

void sara_power(void)
{
  //power down
  nrf_gpio_pin_write(SARA_PWR, 1);
  vTaskDelay(1000);
  nrf_gpio_pin_write(SARA_PWR, 0);
  vTaskDelay(1800);
  nrf_gpio_pin_write(SARA_PWR, 1);
  vTaskDelay(1000);
  

  nrf_gpio_pin_write(SARA_RST, 1);
  vTaskDelay(1000);
  nrf_gpio_pin_write(SARA_RST, 0);
  vTaskDelay(10000);
  nrf_gpio_pin_write(SARA_RST, 1);
  vTaskDelay(1000);

  //power up
  nrf_gpio_pin_write(SARA_PWR, 1);
  nrf_gpio_pin_write(SARA_RST, 1);
  vTaskDelay(1000);
  nrf_gpio_pin_write(SARA_PWR, 0);
  vTaskDelay(1800);
  nrf_gpio_pin_write(SARA_PWR, 1);
  vTaskDelay(1000);
}

void sara_setup(void)
{
  char atCommand[64];

  app_uart_flush();
  _sara.rxBuffer_ptr = &_sara.rxBuffer[0];
  strcpy(atCommand, "AT+CCID\r");
  sara_write(atCommand, strlen(atCommand));
  vTaskDelay(1000);
  sara_read(_sara.rxBuffer, 50);

  app_uart_flush();
  _sara.rxBuffer_ptr = &_sara.rxBuffer[0];
  strcpy(atCommand, "AT+COPS=2\r");
  sara_write(atCommand, strlen(atCommand));
  vTaskDelay(1000);
  sara_read(_sara.rxBuffer, 50);
}

void sara_task(void)
{
  sara_init();
  sara_setup();

  for(;;)
  {

  }
}

void sara_write(char *buffer, char length)
{
  for(int n = 0; n < length; n++)
    while (app_uart_put(buffer[n]) != NRF_SUCCESS);
}

void sara_read(uint8_t *buffer, int length)
{
  uint8_t c;

  for(int n = 0; n < length; n++)
  {
      app_uart_get(&c);
      buffer[n] = c;
  }
}