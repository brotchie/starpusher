#include <math.h>

#include "color.h"
#include "viz.h"

typedef struct {
  vizualization_t viz;
  uint32_t animation_cycle;
  led_segment_t segments[3];
} totem_test_pattern_vizualization_t;

void totem_test_pattern_initialize(totem_test_pattern_vizualization_t *viz,
                                   led_info_t led_info) {
  viz->segments[0] = led_info.segments[0];
  viz->segments[1] = led_info.segments[1];
  viz->segments[2] = led_info.segments[2];
}

void totem_test_pattern_deinitialize(totem_test_pattern_vizualization_t *viz) {}

static rgb_t totem_test_pattern_segment_colors[] = {
    {.r = 255, .g = 0, .b = 0},
    {.r = 0, .g = 255, .b = 0},
    {.r = 0, .g = 0, .b = 255},
};

static rgb_t totem_test_pattern_black = {.r = 0, .g = 0, .b = 0};

void totem_test_pattern_get_pixel_values(
    totem_test_pattern_vizualization_t *viz,
    rgb_t *pixels,
    uint16_t pixel_count) {
  float brightness = fabsf(sinf(viz->animation_cycle / 50.0f));
  for (int segment = 0; segment < 3; segment++) {
    for (int i = viz->segments[segment].start_index;
         i <= viz->segments[segment].end_index;
         i++) {

      if (segment == 0 && i % 2 == 1) {
        pixels[i] = totem_test_pattern_black;
      } else {
        scale_color(
            brightness, totem_test_pattern_segment_colors[segment], &pixels[i]);
      }
    }
  }
}

void totem_test_pattern_tick(totem_test_pattern_vizualization_t *viz,
                             uint32_t animation_cycle) {
  viz->animation_cycle = animation_cycle;
};

REGISTER_VIZUALIZATION(totem_test_pattern);