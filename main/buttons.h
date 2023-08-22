#ifndef BUTTONS_H
#define BUTTONS_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  BUTTON_IDLE,
  BUTTON_PRESS_DETECTED,
  BUTTON_WAIT_FOR_RELEASE,
  BUTTON_RELEASED,
  BUTTON_WAIT_FOR_SECOND_PRESS
} button_state_t;

typedef enum {
  PRESS_NONE,
  PRESS_SINGLE,
  PRESS_DOUBLE,
  PRESS_LONG
} button_press_t;

typedef struct {
  button_state_t state;
  uint32_t last_press_time;
  uint32_t last_release_time;
  button_press_t press_type;
} button_t;

button_press_t
detect_button_press(bool is_pressed, button_t *button, uint32_t current_time);
#endif