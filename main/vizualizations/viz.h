#ifndef VIZ_H
#define VIZ_H

#include "color.h"
#include "common.h"
#include "vizdefs.h"
#include <stdbool.h>
#include <stdlib.h>

#ifdef STARPUSHER_TOTEM
typedef enum {
  POLE = 0,
  STAR1 = 1,
  STAR2 = 2,
} totem_led_segment_t;
#endif

typedef struct {
  uint16_t start_index;
  uint16_t end_index;
} led_segment_t;

typedef struct {
  uint16_t led_count;
  led_segment_t segments[3];
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
DEFINE_VIZUALIZATION(starburst);

#endif