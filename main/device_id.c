#include <driver/gpio.h>
#include <stdbool.h>

#include "device_id.h"

#define DEVICE_ID_DATA GPIO_NUM_35
#define DEVICE_ID_LD GPIO_NUM_12
#define DEVICE_ID_CLK GPIO_NUM_5

static bool device_id_initialized = false;

void device_id_initialize() {
  esp_err_t ret;

  // An 74HC165 has two inputs and three outputs. The LD input
  // when pulled low loads data from 8 x inputs. The CLK input,
  // when strobed, on the rising edge, clocks the next bit out
  // to DATA (Q7 on shift register).
  ret = gpio_set_direction(DEVICE_ID_DATA, GPIO_MODE_INPUT);
  ESP_ERROR_CHECK(ret);
  // We currently have LD on GPIO12, which defaults back to JTAG
  // on reset. Here we reset the pin back to being a GPIO.
  gpio_reset_pin(DEVICE_ID_LD);
  ret = gpio_set_direction(DEVICE_ID_LD, GPIO_MODE_OUTPUT);
  ESP_ERROR_CHECK(ret);
  ret = gpio_set_direction(DEVICE_ID_CLK, GPIO_MODE_OUTPUT);
  ESP_ERROR_CHECK(ret);

  // Start off with load pin high, then strobe down to load data
  // from Device ID DIP switch.
  ret = gpio_set_level(DEVICE_ID_LD, 1);
  ESP_ERROR_CHECK(ret);

  // Start with clock low, each rising edge of the clock moves the
  // next bit into Q7.
  ret = gpio_set_level(DEVICE_ID_CLK, 0);
  ESP_ERROR_CHECK(ret);
}

// DIP switches 1-8 aren't mapped in sequence to the 74HC165 shift
// register (track routing was awkward if we did this), so we correct
// in software.
static const uint8_t bit_shifts[8] = {3, 2, 1, 0, 4, 5, 6, 7};

uint8_t device_id_read() {
  if (!device_id_initialized) {
    device_id_initialize();
    device_id_initialized = true;
  }
  uint8_t device_id = 0;

  // Parallel read in DIP switch state.
  gpio_set_level(DEVICE_ID_LD, 0);
  gpio_set_level(DEVICE_ID_LD, 1);

  // Clock out each bit (active low) and shift to correct bit position.
  for (uint8_t i = 0; i < 8; i++) {
    int level = 1 - gpio_get_level(DEVICE_ID_DATA);
    device_id |= level << bit_shifts[i];
    gpio_set_level(DEVICE_ID_CLK, 1);
    gpio_set_level(DEVICE_ID_CLK, 0);
  }

  return device_id;
}