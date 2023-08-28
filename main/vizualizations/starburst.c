#include "color.h"
#include "viz.h"

#define STAR_LED_COUNT 45
#define STAR_1_START 196
#define STAR_2_START (STAR_1_START + 45)
#define STAR_2_END (STAR_2_START + 45)

typedef struct {
  vizualization_t viz;
  uint32_t animation_cycle;
  double star_sparkle_probability;
} starburst_vizualization_t;

void starburst_initialize(starburst_vizualization_t *viz, led_info_t led_info) {
  viz->star_sparkle_probability = 0.2;
}
void starburst_deinitialize(starburst_vizualization_t *viz) {}

rgb_t star_color = {.r = 255, .g = 255, .b = 0};
rgb_t sparkle_color = {.r = 255, .g = 255, .b = 255};

void starburst_get_pixel_values(starburst_vizualization_t *viz,
                                rgb_t *pixels,
                                uint16_t pixel_count) {
  uint32_t offset = viz->animation_cycle % ((pixel_count / 3) + 200);
  for (uint16_t index = 0; index < STAR_1_START; index++) {
    if (index / 3 == offset) {
      double hue = (double)((index + viz->animation_cycle) % pixel_count) /
                   (double)pixel_count;
      pixels[index] = hsv_to_rgb(hue, 1.0, 1.0);
    } else {
      pixels[index].r *= 0.8;
      pixels[index].g *= 0.8;
      pixels[index].b *= 0.8;
    }
  }

  uint16_t sparkle_mod = 1 / viz->star_sparkle_probability;

  for (uint16_t index = STAR_1_START; index < STAR_2_END; index++) {
    if (offset > 65 && offset < 90) {
      uint16_t ticks_since_star_shine = offset - 65;
      if (ticks_since_star_shine == 5 || ticks_since_star_shine == 8) {
        pixels[index] = sparkle_color;
      } else if (ticks_since_star_shine == 6 || ticks_since_star_shine == 7) {
        pixels[index].r = 0;
        pixels[index].g = 0;
        pixels[index].b = 0;
      } else {

        pixels[index] = star_color;
      }
    } else {
      pixels[index].r *= 0.95;
      pixels[index].g *= 0.95;
      pixels[index].b *= 0.95;
    }
  }
}

void starburst_tick(starburst_vizualization_t *viz, uint32_t animation_cycle) {
  viz->animation_cycle = animation_cycle;
};

REGISTER_VIZUALIZATION(starburst);