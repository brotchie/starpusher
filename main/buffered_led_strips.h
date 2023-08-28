#ifndef BUFFERED_LED_STRIPS_H
#define BUFFERED_LED_STRIPS_H

#include "common.h"
#include <inttypes.h>
#include <stdbool.h>

void buffered_led_strips_initialize_default();
void buffered_led_strips_safe_reconfigure(uint32_t clock_speed_hz);
void buffered_led_strips_update();
void buffered_led_strips_reset(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void buffered_led_strips_set_pixel_value(
    uint8_t port, uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w);

uint8_t buffered_led_strips_acquire_buffers_mutex();
void buffered_led_strips_release_buffers_mutex();
void buffered_led_strips_start_update_task();
void buffered_led_strips_set_vizualiations(bool state);
void buffered_led_strips_next_vizualization();
void buffered_led_strips_previous_vizualization();
void buffered_led_strips_increase_vizualization_brightness();
void buffered_led_strips_decrease_vizualization_brightness();

#ifdef STARPUSHER_TOTEM
void buffered_led_strips_toggle_totem_beacon_mode();
void buffered_led_strips_cycle_star_mode();
void buffered_led_strips_cycle_augmentation();
#endif

#endif