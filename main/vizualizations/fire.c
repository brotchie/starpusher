#include <string.h>

#include "color.h"
#include "viz.h"

typedef struct {
  vizualization_t viz;
  uint16_t fire_palette_size;
  uint16_t fire_noise_size;
  rgb_t *fire_palette;
  uint8_t *fire_noise;
} fire_vizualization_t;

void fire_initialize(fire_vizualization_t *viz, led_info_t led_info) {
  viz->fire_palette_size = 256;
  viz->fire_noise_size = 256;

  viz->fire_palette = malloc(sizeof(rgb_t) * viz->fire_palette_size);
  viz->fire_noise = malloc(sizeof(uint8_t) * viz->fire_noise_size);

  // black to red
  for (uint16_t i = 0; i < 64; ++i) {
    viz->fire_palette[i].r = i * 4;
    viz->fire_palette[i].g = 0;
    viz->fire_palette[i].b = 0;
  }

  // red to orange
  for (uint16_t i = 64; i < 128; ++i) {
    viz->fire_palette[i].r = 255;
    viz->fire_palette[i].g = (i - 64) * 4;
    viz->fire_palette[i].b = 0;
  }

  // orange to yellow
  for (uint16_t i = 128; i < 192; ++i) {
    viz->fire_palette[i].r = 255;
    viz->fire_palette[i].g = 255;
    viz->fire_palette[i].b = (i - 128) * 4;
  }

  // yellow to white
  for (uint16_t i = 192; i < 256; ++i) {
    viz->fire_palette[i].r = 255;
    viz->fire_palette[i].g = 255;
    viz->fire_palette[i].b = 255;
  }

  for (uint16_t i = 0; i < viz->fire_noise_size; i++) {
    viz->fire_noise[i] = rand() % viz->fire_noise_size;
  }
}

void fire_deinitialize(fire_vizualization_t *viz) {
  free(viz->fire_palette);
  free(viz->fire_noise);
}

void fire_get_pixel_values(fire_vizualization_t *viz,
                           rgb_t *pixels,
                           uint16_t pixel_count) {
  for (uint16_t index = 0; index < pixel_count; index++) {

    uint16_t noise_index = (index + (rand() % 3 - 1)) % viz->fire_noise_size;
    uint16_t color_index = viz->fire_noise[noise_index];
    pixels[index] = viz->fire_palette[color_index];
  }
}

void fire_tick(fire_vizualization_t *viz, uint32_t animation_cycle) {
  if (animation_cycle % 10 == 0) {
    memmove(viz->fire_noise, viz->fire_noise + 1, viz->fire_noise_size - 1);
    viz->fire_noise[viz->fire_noise_size - 1] = rand() % viz->fire_palette_size;
  }
};

REGISTER_VIZUALIZATION(fire);