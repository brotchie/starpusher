#ifndef LED_STRIPS_H
#define LED_STRIPS_H

#include "color.h"

void led_strips_start_update_task();
void led_strips_bulk_update(uint8_t port,
                            uint16_t start_index,
                            uint16_t count,
                            rgb_t *colors);

#endif
