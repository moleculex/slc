#include <stdio.h>
#include <string.h>
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "rn2483.h"
#include "sara.h"
#include "gps.h"
#include "FreeRTOS.h"
#include "task.h"
#include "photo.h"
#include "pwm.h"
#include "dali.h"
#include "protocol.h"
#include "flash.h"

void buzzer_task(void)
{
  
  nrf_gpio_cfg_output(BUZZER);
  nrf_gpio_pin_write(BUZZER, 0);


  for(;;)
  {
    //nrf_gpio_pin_write(BUZZER, 1);
    vTaskDelay(1000);
    nrf_gpio_pin_write(BUZZER, 0);
    vTaskDelay(1000);

  }
}

#define BROADCAST_C 0b11111110
#define ON_C 0b00000101
#define _25 64
#define _50 127
#define _75 192
#define _100 254
#define OFF_C 0b00000000

void led_task(void)
{
  nrf_gpio_cfg_output(LED_RED);
  nrf_gpio_cfg_output(LED_GREEN);
  nrf_gpio_cfg_output(LED_BLUE);
  nrf_gpio_cfg_output(LED_POWER);
  nrf_gpio_cfg_output(LED_NETWORK);

  nrf_gpio_pin_write(LED_RED, 0);
  nrf_gpio_pin_write(LED_GREEN, 0);
  nrf_gpio_pin_write(LED_BLUE, 0);
  nrf_gpio_pin_write(LED_POWER, 0);
  nrf_gpio_pin_write(LED_NETWORK, 0);

  dali_init();

  for(;;)
  {
    dali_tx(BROADCAST_C, _100);
    vTaskDelay(2000);
    nrf_gpio_pin_write(LED_RED, 0);
    nrf_gpio_pin_write(LED_GREEN, 0);
    nrf_gpio_pin_write(LED_BLUE, 0);
    nrf_gpio_pin_write(LED_POWER, 0);
    nrf_gpio_pin_write(LED_NETWORK, 0);
    dali_tx(BROADCAST_C, _75);
    vTaskDelay(2000);
    dali_tx(BROADCAST_C, _50);
    vTaskDelay(2000);
    dali_tx(BROADCAST_C, _25);
    vTaskDelay(2000);
    dali_tx(BROADCAST_C, 0);
    vTaskDelay(2000);
    
    /*nrf_gpio_pin_write(LED_RED, 1);
    nrf_gpio_pin_write(LED_GREEN, 1);
    nrf_gpio_pin_write(LED_BLUE, 1);
    nrf_gpio_pin_write(LED_POWER, 1);
    nrf_gpio_pin_write(LED_NETWORK, 1);
    dali_tx(BROADCAST_C, OFF_C);
    vTaskDelay(1000);*/
  }
}

TaskHandle_t  rn2483_task_handle;
TaskHandle_t  buzzer_task_handle;
TaskHandle_t  photo_task_handle;
TaskHandle_t  pwm_task_handle;
TaskHandle_t  led_task_handle;
TaskHandle_t  sara_task_handle;
TaskHandle_t  gps_task_handle;

int main(void)
{
    flash_init();
    vTaskDelay(100);

    //bsp_board_init(BSP_INIT_LEDS); 

    nrf_gpio_cfg_output(TRIAC);

    //twi_init();

    /*uint8_t reg[2] = {0x18, 0x05};
    err_code = nrf_drv_twi_tx(&m_twi, 0x3e, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);*/

    /*reg[0] = 0x1A;
    reg[1] = 0x80;
    err_code = nrf_drv_twi_tx(&m_twi, 0x1f<<1, reg, sizeof(reg), false);*/

    /*reg[0] = 0x0f;
    reg[1] = 0x00;
    err_code = nrf_drv_twi_tx(&m_twi, 0x3e, reg, 2, true);
    while (m_xfer_done == false);
    err_code = nrf_drv_twi_rx(&m_twi, 0x3e, &m_sample, 1);*/
    //while (m_xfer_done == false);

    /*reg[0] = 0x1C;
    reg[1] = 0x30;
    err_code = nrf_drv_twi_tx(&m_twi, 0x1f<<1, reg, sizeof(reg), false);*/

    //kx122_write(0x18, 0x05);

    //kx122_write(0x1A, 0x80); //CNTL3
    //kx122_write(0x1C, 0x30); //INC1
    //kx122_write(0x1F, 0x04);//INC4 Tilt int was 0x05 disabled tilt int
    //kx122_write(0x26, 0xFF);//TTH high thresh 0xff
    //kx122_write(0x27, 0xDA);//hard
    //kx122_write(0x28, 0xF1);//FTD (1/50HZ)*15 > FTD > (1/50HZ)*1 (20ms) duration tap is above threshold H4 H3 H2 H1 H0 L2 L1 L0
    //kx122_write(0x2A, 0x01);//TLT 1/50 * 1 = 20ms sample period
    //kx122_write(0x2B, 0x0A);//TWS 1/50 * 10 = 200ms sample window

    //kx122_write(0x18, 0x85);

    xTaskCreate(buzzer_task, "buzzer_task", configMINIMAL_STACK_SIZE + 200, NULL, 2, &buzzer_task_handle);
    xTaskCreate(photo_task, "photo_task", configMINIMAL_STACK_SIZE + 200, NULL, 2, &photo_task_handle);
    //xTaskCreate(rn2483_task, "rn2483_task", configMINIMAL_STACK_SIZE + 1024, NULL, 2, &rn2483_task_handle);
    xTaskCreate(pwm_task, "pwm_task", configMINIMAL_STACK_SIZE + 200, NULL, 2, &pwm_task_handle);
    xTaskCreate(led_task, "led_task", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_task_handle);
    //xTaskCreate(sara_task, "sara_task", configMINIMAL_STACK_SIZE + 200, NULL, 2, &sara_task_handle);
    xTaskCreate(gps_task, "gps_task", configMINIMAL_STACK_SIZE + 1024, NULL, 2, &gps_task_handle);
    vTaskStartScheduler();

    while (true) {}
}
