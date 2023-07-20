#include "color.h"
#include "viz.h"

typedef struct {
  vizualization_t viz;
  uint32_t animation_cycle;
} rainbow_vizualization_t;

void rainbow_initialize(rainbow_vizualization_t *viz, led_info_t led_info) {}
void rainbow_deinitialize(rainbow_vizualization_t *viz) {}

void rainbow_get_pixel_values(rainbow_vizualization_t *viz,
                              rgb_t *pixels,
                              uint16_t pixel_count) {
  for (uint16_t index = 0; index < pixel_count; index++) {
    double hue = (double)((index + viz->animation_cycle) % pixel_count) /
                 (double)pixel_count;
    pixels[index] = hsv_to_rgb(hue, 1.0, 1.0);
  }
}

void rainbow_tick(rainbow_vizualization_t *viz, uint32_t animation_cycle) {
  viz->animation_cycle = animation_cycle;
};

REGISTER_VIZUALIZATION(rainbow);