#include "buffered_led_strips.h"
#include "buttons.h"
#include "common.h"
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

static const char *TAG = "peripherals";

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

button_t red_button;
button_t green_button;

static void button_poll_task(void *params) {
  TickType_t last_wake_time;
  TickType_t delay_ticks = (1000 / 20) / portTICK_PERIOD_MS;
  last_wake_time = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last_wake_time, delay_ticks);
    button_press_t red_press =
        detect_button_press(!gpio_get_level(DEVICE_ID_BIT1_GPIO),
                            &red_button,
                            xTaskGetTickCount() * portTICK_PERIOD_MS);
    button_press_t green_press =
        detect_button_press(!gpio_get_level(DEVICE_ID_BIT2_GPIO),
                            &green_button,
                            xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (red_press == PRESS_SINGLE) {
      buffered_led_strips_next_vizualization();
    }
    if (green_press == PRESS_SINGLE) {
      buffered_led_strips_previous_vizualization();
    }
    if (red_press == PRESS_LONG) {
      buffered_led_strips_increase_vizualization_brightness();
    }
    if (green_press == PRESS_LONG) {
#ifdef STARPUSHER_TOTEM
      buffered_led_strips_toggle_totem_beacon_mode();
#else
      buffered_led_strips_decrease_vizualization_brightness();
#endif
    }

    if (green_press == PRESS_DOUBLE) {
      buffered_led_strips_cycle_star_mode();
    }

    if (red_press == PRESS_DOUBLE) {
      buffered_led_strips_cycle_augmentation();
    }
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
