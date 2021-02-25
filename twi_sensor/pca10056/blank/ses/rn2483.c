#include <string.h>
#include "rn2483.h"
#include "nrf_uart.h"
#include "app_uart.h"
#include "boards.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_error.h"

void uart_error_handle(app_uart_evt_t * p_event)
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

uint8_t tx[64];
uint8_t rx[64];

static int i = 0;
static uint8_t cr;

void rn2483_rd(uint8_t *buffer, int length)
{
  uint8_t c;

  for(int n = 0; n < length; n++)
  {
    while (app_uart_get(&c) != NRF_SUCCESS);
      buffer[i++] = c;
  }
}

void rn2483_wr(uint8_t *buffer)
{
  for(int n = 0; n < strlen(buffer); n++)
    while (app_uart_put(tx[n]) != NRF_SUCCESS);
}

void rn2483_init(void)
{
  uint32_t err_code;

  nrf_gpio_pin_write(RN2483_RESET, 1);
  vTaskDelay(1000);
  nrf_gpio_pin_write(RN2483_RESET, 0);
  vTaskDelay(1000);
  nrf_gpio_pin_write(RN2483_RESET, 1);
  vTaskDelay(1000);

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
                     uart_error_handle,
                     APP_IRQ_PRIORITY_LOWEST,
                     err_code);

  APP_ERROR_CHECK(err_code);

  app_uart_flush();
  strcpy(tx, "sys reset\r\n");
  rn2483_wr(tx);
  rn2483_rd(rx, 10);

  app_uart_flush();
}

void rn2483_task(void)
{
  rn2483_init();

  for(;;)
  {
    vTaskDelay(1000);
  }
}