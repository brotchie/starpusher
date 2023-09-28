#include <inttypes.h>

#include "buffered_led_strips.h"
#include "common.h"
#include "led_strips.h"
#include "networking.h"
#include "peripherals.h"

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

void app_main(void) {
  led_strips_initialize();
  led_strips_start_update_task();

  esp_err_t ret = gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT);
  ESP_ERROR_CHECK(ret);

  // const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

  // while (1) {
  //  uint32_t data = 0x00050000;
  //  for (uint8_t i = 0; i < 24; i++) {
  //    if (data & 0x80000000) {
  //      gpio_set_level(GPIO_NUM_17, 1);
  //      gpio_set_level(GPIO_NUM_17, 1);
  //      gpio_set_level(GPIO_NUM_17, 0);
  //    } else {
  //      gpio_set_level(GPIO_NUM_17, 1);
  //      gpio_set_level(GPIO_NUM_17, 0);
  //      gpio_set_level(GPIO_NUM_17, 0);
  //    }
  //    data <<= 1;
  //  }
  //  vTaskDelay(xDelay);
  //}
}
// Main entrypoint for Starpushers.
//
// Quick guide of codebase:
//
// - Where is the wire protocol of APA102 LEDs actually clocked
//   out to the LED strips?
//
//   Look a buffered_led_strips_update_task() in buffered_led_strips.c,
//   this runs through each step, copies LED state buffers into DMA,
//   and then, in-parallel, clocks out two strips at a time.
//
//  - Where are UDP packets decoded?
//
//    process_led_packet() in udp_protocol.c is the main parsing code.
//    It works by looking for the magic prefix "BD" or "SPI" to determine
//    the different between list of set_pixel messages ("BD") and a SPI
//    bus config message ("SPI"). The SPI message allows the clock speed
//    of the SPI bus to be set on-the-fly.
//
//  - Where is the Device ID read from the DIP switch?
//
//    See device_id_get() in peripherals.c
//
//  - Where are discovery messages multicasted and to where?
//
//    Constants at the top of networking.c contain the discovery multicast
//    group and port. Discovery packets are constructed and sent in
//    udp_discovery_callback().
//
void old_app_main(void) {
  // Blue indicator LED stays on while Starpusher is receiving
  // UDP packets
  indicator_led_initialize();

  // Initialize 4 x 420 LED strips.
  buffered_led_strips_initialize_default();

  // Read the Starpusher's Device ID from its DIP switch.
  device_id_initialize();
  uint8_t device_id = device_id_get();

#ifdef STARPUSHER_PORTABLE
  buffered_led_strips_reset(0, 0, 0, 0);
#ifndef STARPUSHER_PORTABLE_NETWORKING
  buffered_led_strips_set_vizualiations(1);
#endif
  vizualization_button_initialize();
#else
  if (device_id == WHITE_TEST_PATTERN_DEVICE_ID) {
    buffered_led_strips_reset(255, 255, 255, 255);
  } else if (device_id == RAINBOW_CHASE_TEST_PATTERN_DEVICE_ID) {
    buffered_led_strips_set_vizualiations(1);
    vizualization_button_initialize();
  } else {
    buffered_led_strips_reset(0, 0, 0, 0);
  }
#endif

  // Kick-off the primary LED update task. This is effectively
  // a loop that clocks out pixel buffers to LED strips as
  // fast as possible.
  //
  // The LED update task is pinned to CPU1 (only task running)
  // on that core so it's can run without interruption.
  buffered_led_strips_start_update_task();

// Initialize networking, assigning a Static IP based on
// Device ID, listen for UDP packets, and multicast a discovery
// packet every second.
//
// All networking is configured to be pinned to CPU0 (ESP32
// is dual core).
#if !defined(STARPUSHER_PORTABLE) || defined(STARPUSHER_PORTABLE_NETWORKING)
  if (device_id != WHITE_TEST_PATTERN_DEVICE_ID &&
      device_id != RAINBOW_CHASE_TEST_PATTERN_DEVICE_ID) {
    networking_initialize(device_id);
  }
#endif
}
