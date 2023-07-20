#include <string.h>
#include <math.h>

#include "color.h"
#include "viz.h"

typedef struct {
  vizualization_t viz;
  uint16_t buffer_size;
  rgb_t *buffer;
} lasers_vizualization_t;

void lasers_initialize(lasers_vizualization_t *viz, led_info_t led_info) {
    viz->buffer_size = 64;
    viz->buffer = malloc(sizeof(rgb_t) * viz->buffer_size);
}

void lasers_deinitialize(lasers_vizualization_t *viz) {
    free(viz->buffer);
}

void lasers_get_pixel_values(lasers_vizualization_t *viz,
                              rgb_t *pixels,
                              uint16_t pixel_count) {
  for (uint16_t index = 0; index < pixel_count; index++) {
    pixels[index] = viz->buffer[index % viz->buffer_size];
  }
}

void lasers_tick(lasers_vizualization_t *viz, uint32_t animation_cycle) {
    rgb_t rainbow[] = {
        {.r=255, .g=0, .b=0},
        {.r=255, .g=127, .b=0},
        {.r=255, .g=255, .b=0},
        {.r=0, .g=255, .b=0},
        {.r=0, .g=0, .b=255},
        {.r=75, .g=0, .b=130},
        {.r=148, .g=0, .b=211},
    };
    uint8_t color_count = 7;
    memset(viz->buffer, 0, sizeof(rgb_t) * viz->buffer_size);
    for (uint8_t i = 0; i < color_count; i++) {
        int16_t mag = abs(5*cosf((2 * 3.1415 / 240.0) * animation_cycle));
        int16_t offset = round(mag * sinf((2 * 3.1415 / 60.0) * animation_cycle));
        uint16_t center = (i+1) * 5;
        viz->buffer[(center + offset) % viz->buffer_size] = rainbow[i];
        viz->buffer[(center - offset) % viz->buffer_size] = rainbow[i];
    }

};

REGISTER_VIZUALIZATION(lasers);