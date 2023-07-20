#ifndef VIZ_H
#define VIZ_H

#include "vizdefs.h"
#include "color.h"
#include <stdlib.h>
#include <stdbool.h>

typedef struct {
  uint16_t led_count;
} led_info_t;

struct vizualization_s;

typedef struct vizualization_s {
  void (*initialize)(struct vizualization_s *viz, led_info_t led_info);
  void (*deinitialize)(struct vizualization_s *viz);
  void (*tick)(struct vizualization_s *viz, uint32_t animation_cycle);
  void (*get_pixel_values)(struct vizualization_s *viz,
                           rgb_t *pixels,
                           uint16_t pixel_count);
} vizualization_t;


DEFINE_VIZUALIZATION(alien);
DEFINE_VIZUALIZATION(rainbow);
DEFINE_VIZUALIZATION(fire);
DEFINE_VIZUALIZATION(water);

typedef struct {
    bool race;
} twinkle_config_t;

DEFINE_VIZUALIZATION_WITH_CONFIG(twinkle);
DEFINE_VIZUALIZATION(sinusoidal);
DEFINE_VIZUALIZATION(lightning);
DEFINE_VIZUALIZATION(police);
DEFINE_VIZUALIZATION(lasers);
DEFINE_VIZUALIZATION(perlin);

#endif