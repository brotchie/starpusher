#include <driver/gpio.h>
#include <driver/rmt.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <stdbool.h>
#include <string.h>

#include "indicator_led.h"

#define INDICATOR_LED_GPIO GPIO_NUM_17
#define INDICATOR_LED_RMT_CHANNEL RMT_CHANNEL_0

#define INDICATOR_LED_BUFFER_SIZE 24

#define WS2812B_T0H 16
#define WS2812B_T0L 34
#define WS2812B_T1H 32
#define WS2812B_T1L 18

static TaskHandle_t indicator_led_task_handle;
static SemaphoreHandle_t indicator_led_mutex;

static rgb_t led_color = {.r = 0, .g = 0, .b = 0};

static rmt_item32_t indicator_led_buffer[INDICATOR_LED_BUFFER_SIZE];

static void indicator_led_task(void *params) {
  const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
  uint8_t data[3] = {0, 0, 0};
  for (;;) {
    if (xSemaphoreTake(indicator_led_mutex, portMAX_DELAY) == pdTRUE) {
      data[0] = led_color.g;
      data[1] = led_color.r;
      data[2] = led_color.b;
      xSemaphoreGive(indicator_led_mutex);
    }

    size_t index = 0;

    for (uint8_t i = 0; i < 8; i++) {
      bool bit_is_set = (data[0] & (1 << (7 - i)));
      indicator_led_buffer[index] =
          bit_is_set ? (rmt_item32_t){{{WS2812B_T1H, 1, WS2812B_T1L, 0}}}
                     : (rmt_item32_t){{{WS2812B_T0H, 1, WS2812B_T0L, 0}}};
      index++;
    }
    for (uint8_t i = 0; i < 8; i++) {
      bool bit_is_set = (data[1] & (1 << (7 - i)));
      indicator_led_buffer[index] =
          bit_is_set ? (rmt_item32_t){{{WS2812B_T1H, 1, WS2812B_T1L, 0}}}
                     : (rmt_item32_t){{{WS2812B_T0H, 1, WS2812B_T0L, 0}}};
      index++;
    }
    for (uint8_t i = 0; i < 8; i++) {
      bool bit_is_set = (data[2] & (1 << (7 - i)));
      indicator_led_buffer[index] =
          bit_is_set ? (rmt_item32_t){{{WS2812B_T1H, 1, WS2812B_T1L, 0}}}
                     : (rmt_item32_t){{{WS2812B_T0H, 1, WS2812B_T0L, 0}}};
      index++;
    }

    ESP_ERROR_CHECK(rmt_write_items(INDICATOR_LED_RMT_CHANNEL,
                                    indicator_led_buffer,
                                    INDICATOR_LED_BUFFER_SIZE,
                                    false));
    ESP_ERROR_CHECK(rmt_wait_tx_done(INDICATOR_LED_RMT_CHANNEL, portMAX_DELAY));

    vTaskDelay(xDelay);
  }
}

void indicator_led_initialize() {
  indicator_led_mutex = xSemaphoreCreateMutex();

  rmt_config_t config = {.rmt_mode = RMT_MODE_TX,
                         .channel = INDICATOR_LED_RMT_CHANNEL,
                         .gpio_num = INDICATOR_LED_GPIO,
                         .mem_block_num = 3,
                         .tx_config.loop_en = false,
                         .tx_config.carrier_en = false,
                         .tx_config.idle_output_en = true,
                         .tx_config.idle_level = 0,
                         .clk_div = 2};

  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

  xTaskCreatePinnedToCore(indicator_led_task,
                          "indicator_led_task",
                          4096,
                          NULL,
                          1 /* Priority */,
                          &indicator_led_task_handle,
                          0 /* CPU affinity */);
}

void indicator_led_set(rgb_t color) {
  if (xSemaphoreTake(indicator_led_mutex, portMAX_DELAY) == pdTRUE) {
    led_color = color;
    xSemaphoreGive(indicator_led_mutex);
  }
}