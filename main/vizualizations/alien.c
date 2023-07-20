#include "viz.h"
#include <stdio.h>
#include <string.h>

typedef struct {
  vizualization_t viz;
  rgb_t *alien_buffer;
  uint16_t alien_buffer_pixel_count;
  // Per-frame probability of a pixel flash yellow for one frame.
  double yellow_flash_probability;
  // Probability of spawning a green pixel per frame.
  double green_spawn_probability;
  // Decay green pixels at this rate.
  double green_decay;
  // Shift the buffer one pixel to the left every number of ticks.
  uint16_t buffer_shift_ticks;
} alien_vizualization_t;

void alien_initialize(alien_vizualization_t *viz, led_info_t led_info) {
  viz->alien_buffer_pixel_count = led_info.led_count;
  viz->alien_buffer = malloc(sizeof(rgb_t) * viz->alien_buffer_pixel_count);
  viz->yellow_flash_probability = 0.008;
  viz->green_spawn_probability = 0.1;
  viz->green_decay = 0.95;
  viz->buffer_shift_ticks = 30;
}

void alien_deinitialize(alien_vizualization_t *viz) {
  if (viz->alien_buffer != NULL) {
    free(viz->alien_buffer);
  }
}

rgb_t yellow = {.r = 255, .g = 255, .b = 0};
rgb_t green = {.r = 0, .g = 255, .b = 0};

void alien_get_pixel_values(alien_vizualization_t *viz,
                            rgb_t *pixels,
                            uint16_t pixel_count) {
  uint16_t flash_mod = 1 / viz->yellow_flash_probability;
  for (uint16_t index = 0; index < pixel_count; index++) {
    rgb_t color;
    if (rand() % flash_mod == 0) {
      color = yellow;
    } else {
      color = viz->alien_buffer[index % viz->alien_buffer_pixel_count];
    }
    pixels[index] = color;
  }
}

void alien_tick(alien_vizualization_t *viz, uint32_t animation_cycle) {
  uint16_t spawn_mod = 1 / viz->green_spawn_probability;
  if (rand() % spawn_mod == 0) {
    viz->alien_buffer[rand() % viz->alien_buffer_pixel_count] = green;
  }
  for (uint16_t index = 0; index < viz->alien_buffer_pixel_count; index++) {
    viz->alien_buffer[index].r *= viz->green_decay;
    viz->alien_buffer[index].g *= viz->green_decay;
    viz->alien_buffer[index].b *= viz->green_decay;
  }
  if (animation_cycle % viz->buffer_shift_ticks == 0) {
    rgb_t first = viz->alien_buffer[0];
    memmove(viz->alien_buffer,
            &viz->alien_buffer[1],
            sizeof(rgb_t) * (viz->alien_buffer_pixel_count - 1));
    viz->alien_buffer[viz->alien_buffer_pixel_count - 1] = first;
  }
}

REGISTER_VIZUALIZATION(alien);