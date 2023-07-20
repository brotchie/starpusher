#ifndef COLOR_H
#define COLOR_H

#include <stdint.h>

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} rgb_t;

rgb_t hsv_to_rgb(float h, float s, float v);
void fire_palette_initialize();
void fire_palette_update_noise();
rgb_t fire_palette_get_pixel_value(uint16_t index);

void water_palette_initialize();
rgb_t water_palette_get_pixel_value(uint16_t index);

void heartbeat_palette_initialize();
rgb_t heartbeat_palette_get_pixel_value(uint16_t index);

rgb_t twinkle_buffer_get_pixel_value(uint16_t index);
void twinkle_buffer_tick();

rgb_t alien_buffer_get_pixel_value(uint16_t index);
void alien_buffer_tick(uint16_t animation_cycle);

rgb_t prob_weight_buffer_get_pixel_value(uint16_t index);
void prob_weight_buffer_tick(uint16_t animation_cycle);

rgb_t color_lightning_get_pixel_value(uint16_t index);
void color_lightning_tick(uint16_t animation_cycle);
#endif