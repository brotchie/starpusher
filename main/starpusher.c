#include <inttypes.h>

#include "device_id.h"
#include "indicator_led.h"
#include "led_strips.h"
#include "networking.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

static const char *TAG = "app_main";

// LED Colors:
//  - Blue: Startup,
//  - Magenta: No network,
//  - Green: OK and receiving UDP packets,
//  - Red: Watchdog tripped, no UDP packets received in a while.

void app_main(void) {
  indicator_led_initialize();
  rgb_t led_color = {.r = 0, .g = 0, .b = 128};
  indicator_led_set(led_color);

  uint8_t device_id = device_id_read();

  ESP_LOGI(TAG, "Device id: %d", device_id);

  led_strips_start_update_task();

  networking_initialize(device_id);
}