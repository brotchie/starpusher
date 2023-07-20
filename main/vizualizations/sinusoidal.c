#include <math.h>

#include "color.h"
#include "viz.h"

typedef struct {
  vizualization_t viz;
  uint16_t prob_weight_buffer_size;
  int16_t *prob_weight_buffer;
  double positive_target_hue;
  double negative_target_hue;
  double positive_hue;
  double negative_hue;
} sinusoidal_vizualization_t;

void sinusoidal_initialize(sinusoidal_vizualization_t *viz, led_info_t led_info) {
  viz->prob_weight_buffer_size = 64;
  viz->positive_target_hue = 0.75;
  viz->negative_target_hue = 0.25;
  viz->positive_hue = 0.75;
  viz->negative_hue = 0.25;

  viz->prob_weight_buffer =
      malloc(sizeof(int16_t) * viz->prob_weight_buffer_size);
}
void sinusoidal_deinitialize(sinusoidal_vizualization_t *viz) {
  free(viz->prob_weight_buffer);
}

const rgb_t black = {.r = 0, .g = 0, .b = 0};

void sinusoidal_get_pixel_values(sinusoidal_vizualization_t *viz,
                               rgb_t *pixels,
                               uint16_t pixel_count) {
  for (uint16_t index = 0; index < pixel_count; index++) {
    int16_t weight =
        viz->prob_weight_buffer[index % viz->prob_weight_buffer_size];
    rgb_t color = hsv_to_rgb(viz->positive_hue, 1.0, 1.0);
    if (weight < 0) {
      weight = -weight;
      color = hsv_to_rgb(viz->negative_hue, 1.0, 1.0);
    }
    if (weight != 255 && rand() % (255 - weight) != 0) {
      color = black;
    }
    pixels[index] = color;
  }
}

void sinusoidal_tick(sinusoidal_vizualization_t *viz, uint32_t animation_cycle) {
  if (animation_cycle % 120 == 0) {
    viz->negative_target_hue = (double)(rand() % 255) / 255.0;
    viz->positive_target_hue = (double)(rand() % 255) / 255.0;
  }
  float negative_target_delta = viz->negative_target_hue - viz->negative_hue;
  float positive_target_delta = viz->positive_target_hue - viz->positive_hue;
  viz->negative_hue += 0.01 * negative_target_delta;
  viz->positive_hue += 0.01 * positive_target_delta;
  for (uint16_t i = 0; i < viz->prob_weight_buffer_size; i++) {
    viz->prob_weight_buffer[i] =
        255 * sinf(0.02 * animation_cycle +
                   2 * 3.1415 * (float)i / (float)viz->prob_weight_buffer_size);
  }
};

REGISTER_VIZUALIZATION(sinusoidal);