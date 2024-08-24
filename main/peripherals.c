#include "buffered_led_strips.h"
#include "buttons.h"
#include "common.h"
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

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
        detect_button_press(!gpio_get_level(BUTTON1_GPIO),
                            &red_button,
                            xTaskGetTickCount() * portTICK_PERIOD_MS);
    button_press_t green_press =
        detect_button_press(!gpio_get_level(BUTTON2_GPIO),
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
  esp_err_t ret;
  ret = gpio_set_direction(BUTTON1_GPIO, GPIO_MODE_INPUT);
  ESP_ERROR_CHECK(ret);
  ret = gpio_set_direction(BUTTON2_GPIO, GPIO_MODE_INPUT);
  ESP_ERROR_CHECK(ret);
  ret = gpio_pullup_en(BUTTON1_GPIO);
  ESP_ERROR_CHECK(ret);
  ret = gpio_pullup_en(BUTTON2_GPIO);
  ESP_ERROR_CHECK(ret);

  xTaskCreatePinnedToCore(button_poll_task,
                          "button_poll_task",
                          4096,
                          NULL,
                          4,
                          &button_poll_task_handle,
                          0);
}
