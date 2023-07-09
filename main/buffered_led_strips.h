#ifndef BUFFERED_LED_STRIPS_H
#define BUFFERED_LED_STRIPS_H

#include <inttypes.h>

void buffered_led_strips_initialize_default();
void buffered_led_strips_initialize(uint32_t clock_speed_hz);
void buffered_led_strips_deinitialize();
void buffered_led_strips_update();
void buffered_led_strips_reset();
void buffered_led_strips_set_pixel_value(
    uint8_t port, uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w);

#endif