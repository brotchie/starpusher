#include "color.h"
#include "viz.h"

typedef struct {
  vizualization_t viz;
  uint16_t chunk_length;
  uint8_t active_chunk;
  uint32_t switch_ticks;
} police_vizualization_t;

void police_initialize(police_vizualization_t *viz, led_info_t led_info) {
    viz->chunk_length = 5;
    viz->active_chunk = 0;
    viz->switch_ticks = 10;
}
void police_deinitialize(police_vizualization_t *viz) {}

void police_get_pixel_values(police_vizualization_t *viz,
                              rgb_t *pixels,
                              uint16_t pixel_count) {
  for (uint16_t index = 0; index < pixel_count; index++) {
    rgb_t color = {.r=0, .g=0, .b=0};
    if ((index / viz->chunk_length) % 2 == viz->active_chunk) {
        color.b = 255;
    } else {
        color.r = 255;
    }
    pixels[index]= color;
  }
}

void police_tick(police_vizualization_t *viz, uint32_t animation_cycle) {
    viz->active_chunk = (animation_cycle / viz->switch_ticks) % 2;
};

REGISTER_VIZUALIZATION(police);