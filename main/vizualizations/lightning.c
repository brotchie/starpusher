#include "color.h"
#include "viz.h"

typedef struct {
  vizualization_t viz;
  rgb_t last_color;
  rgb_t current_color;
  uint16_t ticks_since_last_trigger;
  uint16_t second_trigger_countdown;
} lightning_vizualization_t;

void lightning_initialize(lightning_vizualization_t *viz, led_info_t led_info) {
  viz->ticks_since_last_trigger = 0;
  viz->second_trigger_countdown = 0;
}
void lightning_deinitialize(lightning_vizualization_t *viz) {}

void lightning_get_pixel_values(lightning_vizualization_t *viz,
                                rgb_t *pixels,
                                uint16_t pixel_count) {
  const rgb_t white = {.r = 255, .g = 255, .b = 255};
  const rgb_t black = {.r = 0, .g = 0, .b = 0};

  rgb_t color;

  if (viz->ticks_since_last_trigger == 5 ||
      viz->ticks_since_last_trigger == 8) {
    color = white;
  } else if (viz->ticks_since_last_trigger == 6 ||
             viz->ticks_since_last_trigger == 7) {
    color = black;
  } else {
    color = viz->current_color;
    ;
  }
  for (uint16_t index = 0; index < pixel_count; index++) {
    pixels[index] = color;
  }
}

void lightning_tick(lightning_vizualization_t *viz, uint32_t animation_cycle) {
  viz->ticks_since_last_trigger++;
  viz->current_color.r *= 0.95;
  viz->current_color.g *= 0.95;
  viz->current_color.b *= 0.95;
  if (viz->second_trigger_countdown > 0) {
    viz->second_trigger_countdown--;
  }
  if (viz->second_trigger_countdown == 1) {
    viz->current_color = viz->last_color;
    viz->ticks_since_last_trigger = 0;
  } else if (viz->second_trigger_countdown == 0 &&
             rand() % (200 - viz->ticks_since_last_trigger) == 0) {
    viz->ticks_since_last_trigger = 0;
    viz->last_color = hsv_to_rgb((double)(rand() % 255) / 255.0, 1.0, 1.0);
    viz->current_color = viz->last_color;
    viz->second_trigger_countdown = rand() % 30 + 15;
  }
};

REGISTER_VIZUALIZATION(lightning);