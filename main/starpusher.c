#include <inttypes.h>

#include "buffered_led_strips.h"
#include "networking.h"
#include "peripherals.h"

void app_main(void) {
  indicator_led_initialize();
  buffered_led_strips_initialize_default();

  device_id_initialize();
  uint8_t device_id = device_id_get();

  networking_initialize(device_id);
}
