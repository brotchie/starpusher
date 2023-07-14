#ifndef VIZ_H
#define VIZ_H

#include "color.h"

typedef struct {
    void (*initialize)();
    void (*deinitialize)();
    rgb_t (*get_pixel_value)(uint16_t index, );
} visualization_t;

#endif