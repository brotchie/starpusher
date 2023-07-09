#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <inttypes.h>

// Initialize the GPIO driving the indicator LED.
void indicator_led_initialize();

// Sets the indicator LED on (1) or off (0).
void indicator_led_set(uint8_t value);

// Initialize the GPIOs reading the device id from DIP switch.
void device_id_initialize();

// Reads the device's id from the current DIP switch.
uint8_t device_id_get();

#endif