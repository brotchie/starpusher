#include "color.h"
#include "viz.h"
#include <stdbool.h>

typedef struct {
  vizualization_t viz;
  twinkle_config_t config;
  uint16_t twinkle_buffer_size;
  rgb_t *twinkle_buffer;
  uint32_t animation_cycle;
  double pixel_spawn_probability;
  double pixel_decay;
} twinkle_vizualization_t;

void twinkle_initialize(twinkle_vizualization_t *viz, led_info_t led_info) {
  viz->twinkle_buffer_size = 40;
  viz->pixel_spawn_probability = 0.1;
  viz->pixel_decay = 0.95;
  viz->twinkle_buffer = malloc(sizeof(rgb_t) * viz->twinkle_buffer_size);
}
void twinkle_deinitialize(twinkle_vizualization_t *viz) {
  free(viz->twinkle_buffer);
}

void twinkle_get_pixel_values(twinkle_vizualization_t *viz,
                              rgb_t *pixels,
                              uint16_t pixel_count) {
  for (uint16_t index = 0; index < pixel_count; index++) {
    uint16_t offset = viz->config.race ? viz->animation_cycle : 0;
    pixels[index] =
        viz->twinkle_buffer[(index + offset) % viz->twinkle_buffer_size];
  }
}

void twinkle_tick(twinkle_vizualization_t *viz, uint32_t animation_cycle) {
  viz->animation_cycle = animation_cycle;
  uint16_t spawn_mod = 1 / viz->pixel_spawn_probability;
  if (rand() % spawn_mod == 0) {
    viz->twinkle_buffer[rand() % viz->twinkle_buffer_size] =
        hsv_to_rgb((double)(rand() % 256) / 256.0, 1.0, 1.0);
  }
  for (uint16_t i = 0; i < viz->twinkle_buffer_size; i++) {
    viz->twinkle_buffer[i].r *= viz->pixel_decay;
    viz->twinkle_buffer[i].g *= viz->pixel_decay;
    viz->twinkle_buffer[i].b *= viz->pixel_decay;
  }
};

REGISTER_VIZUALIZATION_WITH_CONFIG(twinkle);