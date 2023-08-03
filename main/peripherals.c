#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include "buffered_led_strips.h"


#define DEVICE_ID_BIT1_GPIO GPIO_NUM_39
#define DEVICE_ID_BIT2_GPIO GPIO_NUM_36
#define DEVICE_ID_BIT3_GPIO GPIO_NUM_12
#define DEVICE_ID_BIT4_GPIO GPIO_NUM_35

#define INDICATOR_LED_GPIO GPIO_NUM_17

void indicator_led_initialize() {
  esp_err_t ret = gpio_set_direction(INDICATOR_LED_GPIO, GPIO_MODE_OUTPUT);
  ESP_ERROR_CHECK(ret);
}

void indicator_led_set(uint8_t value) {
  esp_err_t ret = gpio_set_level(INDICATOR_LED_GPIO, value);
  ESP_ERROR_CHECK(ret);
}

void device_id_initialize() {
  esp_err_t ret;
  ret = gpio_set_direction(DEVICE_ID_BIT1_GPIO, GPIO_MODE_INPUT);
  ESP_ERROR_CHECK(ret);
  ret = gpio_set_direction(DEVICE_ID_BIT2_GPIO, GPIO_MODE_INPUT);
  ESP_ERROR_CHECK(ret);
  ret = gpio_set_direction(DEVICE_ID_BIT3_GPIO, GPIO_MODE_INPUT);
  ESP_ERROR_CHECK(ret);
  ret = gpio_set_direction(DEVICE_ID_BIT4_GPIO, GPIO_MODE_INPUT);
  ESP_ERROR_CHECK(ret);
  ret = gpio_set_pull_mode(DEVICE_ID_BIT3_GPIO, GPIO_PULLUP_ONLY);
  ESP_ERROR_CHECK(ret);
  ret = gpio_pullup_en(DEVICE_ID_BIT3_GPIO);
  ESP_ERROR_CHECK(ret);
}

uint8_t device_id_get() {
  return (!gpio_get_level(DEVICE_ID_BIT1_GPIO)) |
         (!gpio_get_level(DEVICE_ID_BIT2_GPIO) << 1) |
         (!gpio_get_level(DEVICE_ID_BIT3_GPIO) << 2) |
         (!gpio_get_level(DEVICE_ID_BIT4_GPIO) << 3);
}

TaskHandle_t button_poll_task_handle;
int lastButtonState = 0;
int lastTriggeredButtonState = 0;

static void button_poll_task(void *params) {
  TickType_t last_wake_time;
  TickType_t delay_ticks = (1000 / 20) / portTICK_PERIOD_MS;
  last_wake_time = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last_wake_time, delay_ticks);
    int buttonState = !gpio_get_level(DEVICE_ID_BIT1_GPIO);

    if (buttonState == lastButtonState && buttonState != lastTriggeredButtonState) {
      lastTriggeredButtonState = buttonState;
      if (buttonState == 1) {
        buffered_led_strips_next_vizualization();
      }
    }
    lastButtonState = buttonState;
  }
}


void vizualization_button_initialize() {
  xTaskCreatePinnedToCore(button_poll_task,
                          "button_poll_task",
                          4096,
                          NULL,
                          4,
                          &button_poll_task_handle,
                          0);
}
