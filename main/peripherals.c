#include "driver/gpio.h"

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