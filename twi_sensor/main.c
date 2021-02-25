#include <stdio.h>
#include <string.h>
#include "boards.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "rn2483.h"
#include "FreeRTOS.h"
#include "task.h"
#include "photo.h"
#include "pwm.h"


#define TWI_INSTANCE_ID     0


#define LM75B_ADDR          (0x90U >> 1)

#define LM75B_REG_TEMP      0x00U
#define LM75B_REG_CONF      0x01U
#define LM75B_REG_THYST     0x02U
#define LM75B_REG_TOS       0x03U


#define NORMAL_MODE 0U


static volatile bool m_xfer_done = false;


static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);


static uint8_t m_sample[256];                        

void LM75B_set_mode(void)
{
    ret_code_t err_code;

    /* Writing to LM75B_REG_CONF "0" set temperature sensor in NORMAL mode. */
    uint8_t reg[2] = {LM75B_REG_CONF, NORMAL_MODE};
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    /* Writing to pointer byte. */
    reg[0] = LM75B_REG_TEMP;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, 1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}


__STATIC_INLINE void data_handler(uint8_t temp)
{
    //NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}


void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = 19,
       .sda                = 20,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


static void read_sensor_data()
{
    m_xfer_done = false;

    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, 0x42, &m_sample, 128);
    //ret_code_t err_code = nrf_drv_twi_rx(&m_twi, 0x1f, &m_sample, 10);
    APP_ERROR_CHECK(err_code);
}

void buzzer_task(void)
{
  nrf_gpio_cfg_output(BUZZER);
  nrf_gpio_cfg_output(GPS_RESET);

  nrf_gpio_pin_write(BUZZER, 0);
  nrf_gpio_pin_write(GPS_RESET, 1);
  vTaskDelay(1000);
  nrf_gpio_pin_write(GPS_RESET, 0);
  vTaskDelay(1000);
  nrf_gpio_pin_write(GPS_RESET, 1);

  for(;;)
  {
    //nrf_gpio_pin_write(BUZZER, 1);
    vTaskDelay(1000);
    nrf_gpio_pin_write(BUZZER, 0);
    vTaskDelay(1000);
    read_sensor_data();
  }
}

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


  for(;;)
  {
    nrf_gpio_pin_write(LED_RED, 0);
    nrf_gpio_pin_write(LED_GREEN, 0);
    nrf_gpio_pin_write(LED_BLUE, 0);
    nrf_gpio_pin_write(LED_POWER, 0);
    nrf_gpio_pin_write(LED_NETWORK, 0);
    vTaskDelay(1000);
    nrf_gpio_pin_write(LED_RED, 1);
    nrf_gpio_pin_write(LED_GREEN, 1);
    nrf_gpio_pin_write(LED_BLUE, 1);
    nrf_gpio_pin_write(LED_POWER, 1);
    nrf_gpio_pin_write(LED_NETWORK, 1);
    vTaskDelay(1000);
  }
}

TaskHandle_t  rn2483_task_handle;
TaskHandle_t  buzzer_task_handle;
TaskHandle_t  photo_task_handle;
TaskHandle_t  pwm_task_handle;
TaskHandle_t  led_task_handle;

int main(void)
{
    ret_code_t err_code;

    //bsp_board_init(BSP_INIT_LEDS); 

    nrf_gpio_cfg_output(TRIAC);

    twi_init();

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

    xTaskCreate(buzzer_task, "buzzer_task", configMINIMAL_STACK_SIZE + 1024, NULL, 2, &buzzer_task_handle);
    xTaskCreate(photo_task, "photo_task", configMINIMAL_STACK_SIZE + 200, NULL, 2, &photo_task_handle);
    xTaskCreate(rn2483_task, "rn2483_task", configMINIMAL_STACK_SIZE + 1024, NULL, 2, &rn2483_task_handle);
    xTaskCreate(pwm_task, "pwm_task", configMINIMAL_STACK_SIZE + 200, NULL, 2, &pwm_task_handle);
    xTaskCreate(led_task, "led_task", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_task_handle);
    vTaskStartScheduler();

    while (true)
    {
        //vTaskDelay(500);


        //read_sensor_data();
    }
}
