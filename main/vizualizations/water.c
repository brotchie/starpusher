#include <string.h>

#include "color.h"
#include "viz.h"

typedef struct {
  vizualization_t viz;
  uint16_t water_palette_size;
  rgb_t *water_palette;
  uint32_t animation_cycle;
} water_vizualization_t;

void water_initialize(water_vizualization_t *viz, led_info_t led_info) {
  viz->water_palette_size = 256;

  viz->water_palette = malloc(sizeof(rgb_t) * viz->water_palette_size);

  for (int i = 0; i < viz->water_palette_size / 2; i++) {
    viz->water_palette[i].r = 0; // Keep red component low for water
    viz->water_palette[i].g = (i < 128) ? i << 1 : 255 - ((i - 128) << 1);
    viz->water_palette[i].b =
        (i < 64) ? i << 3 : 255; // Keep blue high for water effect

    viz->water_palette[viz->water_palette_size - 1 - i].r =
        0; // Keep red component low for water
    viz->water_palette[viz->water_palette_size - 1 - i].g =
        (i < 128) ? i << 1 : 255 - ((i - 128) << 1);
    viz->water_palette[viz->water_palette_size - 1 - i].b =
        (i < 64) ? i << 3 : 255; // Keep blue high for water effect
  }
}

void water_deinitialize(water_vizualization_t *viz) {
  free(viz->water_palette);
}

const rgb_t white = {.r = 255, .g = 255, .b = 255};

void water_get_pixel_values(water_vizualization_t *viz,
                            rgb_t *pixels,
                            uint16_t pixel_count) {
  for (uint16_t index = 0; index < pixel_count; index++) {
    rgb_t color;
    if (rand() % 32 == 0) {
      color = white;
    } else {
      color = viz->water_palette[(index + viz->animation_cycle) %
                                 viz->water_palette_size];
    }
    pixels[index] = color;
  }
}

void water_tick(water_vizualization_t *viz, uint32_t animation_cycle) {
  viz->animation_cycle = animation_cycle;
};

REGISTER_VIZUALIZATION(water);