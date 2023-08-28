#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "buttons.h"

#define DEBOUNCE_TIME_MS 20
#define DOUBLE_PRESS_TIME_MS 200
#define LONG_PRESS_TIME_MS 1000

button_press_t
detect_button_press(bool is_pressed, button_t *button, uint32_t current_time) {

  switch (button->state) {
  case BUTTON_IDLE:
    if (is_pressed) {
      button->last_press_time = current_time;
      button->state = BUTTON_PRESS_DETECTED;
    }
    break;

  case BUTTON_PRESS_DETECTED:
    if (!is_pressed) {
      button->last_release_time = current_time;
      if (current_time - button->last_press_time < LONG_PRESS_TIME_MS) {
        button->state = BUTTON_WAIT_FOR_SECOND_PRESS;
      } else {
        button->state = BUTTON_IDLE;
        button->press_type = PRESS_LONG;
      }
    } else if (current_time - button->last_press_time >= LONG_PRESS_TIME_MS) {
      button->state = BUTTON_WAIT_FOR_RELEASE;
      button->press_type = PRESS_LONG;
    }
    break;

  case BUTTON_WAIT_FOR_RELEASE:
    if (!is_pressed) {
      button->state = BUTTON_IDLE;
    }
    break;

  case BUTTON_WAIT_FOR_SECOND_PRESS:
    if (is_pressed &&
        (current_time - button->last_release_time <= DOUBLE_PRESS_TIME_MS)) {
      button->state = BUTTON_WAIT_FOR_RELEASE;
      button->press_type = PRESS_DOUBLE;
    } else if (current_time - button->last_release_time >
               DOUBLE_PRESS_TIME_MS) {
      button->state = BUTTON_IDLE;
      button->press_type = PRESS_SINGLE;
    }
    break;
  default:
    break;
  }

  button_press_t result = button->press_type;
  button->press_type = PRESS_NONE;
  return result;
}