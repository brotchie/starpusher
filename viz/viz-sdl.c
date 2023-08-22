#include "SDL.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "buttons.h"
#include "color.h"
#include "viz.h"

#define IP_ADDRESS "127.0.0.1"
#define PORT 6868
#define MAX_UDP_BUFFER_SIZE 1472
#define LED_COUNT 100
#define MAX_VIZUALIZATIONS 20
#define VIZUALIZATION_PREVIEW_SIZE 20

const size_t message_size = 7;
uint8_t udp_buffer[MAX_UDP_BUFFER_SIZE];
uint8_t *buffer_pos = udp_buffer;

button_t red_button, green_button;

vizualization_t *vizualizations[MAX_VIZUALIZATIONS];
vizualization_t *viz;

typedef enum {
  APP_MODE_NORMAL,
  APP_MODE_CONFIG,
} app_mode_t;

typedef enum {
  CONFIG_VAR_GLOBAL_BRIGHTNESS = 0,
  CONFIG_VAR_ENABLED_VIZUALIZATIONS = 1,
  CONFIG_VAR_END = 2,
} config_var_t;

typedef struct {
  app_mode_t mode;
  config_var_t config_var;

  uint8_t global_brightness;

  uint8_t selected_viz;
  uint8_t enabled_vizualizations[MAX_VIZUALIZATIONS];
  vizualization_t *preview_viz;
} app_state_t;

typedef struct {
  vizualization_t viz;
  app_state_t state;
} app_vizualization_t;

uint32_t config_cycle = 0;

void config_initialize(app_vizualization_t *viz, led_info_t led_info) {
  if (viz->state.preview_viz == NULL) {
    viz->state.preview_viz = vizualizations[viz->state.selected_viz];
    led_info_t preview_info = {.led_count = VIZUALIZATION_PREVIEW_SIZE};
    viz->state.preview_viz->initialize(viz->state.preview_viz, preview_info);
  }
}

void config_deinitialize(app_vizualization_t *viz) {
  if (viz->state.preview_viz != NULL) {
    viz->state.preview_viz->deinitialize(viz->state.preview_viz);
  }
}

void config_tick(app_vizualization_t *viz, uint32_t animation_cycle) {
  config_cycle = animation_cycle;
  viz->state.preview_viz->tick(viz->state.preview_viz, animation_cycle);
}

void config_get_pixel_values(app_vizualization_t *viz,
                             rgb_t *pixels,
                             uint16_t pixel_count) {
  if (viz->state.config_var == CONFIG_VAR_GLOBAL_BRIGHTNESS) {
    for (int i = 0; i < pixel_count; i++) {
      if ((i % 8) < viz->state.global_brightness + 1) {
        pixels[i].r = 255;
        pixels[i].g = 255;
        pixels[i].b = 255;
      } else {
        pixels[i].r = 0;
        pixels[i].g = 0;
        pixels[i].b = 0;
      }
    }
  } else if (viz->state.config_var == CONFIG_VAR_ENABLED_VIZUALIZATIONS) {
    viz->state.preview_viz->get_pixel_values(viz->state.preview_viz,
                                             &pixels[MAX_VIZUALIZATIONS],
                                             VIZUALIZATION_PREVIEW_SIZE);
    for (int i = 0; i < pixel_count; i++) {
      uint16_t index = i % (MAX_VIZUALIZATIONS + VIZUALIZATION_PREVIEW_SIZE);
      if (index >= MAX_VIZUALIZATIONS) {
        pixels[i] = pixels[index];
      } else if (vizualizations[index] != NULL) {
        if (viz->state.enabled_vizualizations[index]) {
          pixels[i].r = 0;
          pixels[i].g = 255;
          pixels[i].b = 0;
        } else {
          pixels[i].r = 255;
          pixels[i].g = 0;
          pixels[i].b = 0;
        }
      }
      if (index == viz->state.selected_viz && (config_cycle / 15) % 2 == 0) {
        pixels[i].r = 0;
        pixels[i].g = 0;
        pixels[i].b = 255;
      }
    }
  }
}

void config_increment_var(app_state_t *state) {
  if (state->config_var == CONFIG_VAR_GLOBAL_BRIGHTNESS) {
    state->global_brightness++;
    state->global_brightness %= 8;
  } else if (state->config_var == CONFIG_VAR_ENABLED_VIZUALIZATIONS) {
    state->selected_viz++;
    if (vizualizations[state->selected_viz] == NULL) {
      state->selected_viz = 0;
    }
    if (state->preview_viz) {
      state->preview_viz->deinitialize(state->preview_viz);
    }
    state->preview_viz = vizualizations[state->selected_viz];
    led_info_t preview_info = {.led_count = VIZUALIZATION_PREVIEW_SIZE};
    state->preview_viz->initialize(state->preview_viz, preview_info);
  }
}

void config_decrement_var(app_state_t *state) {
  if (state->config_var == CONFIG_VAR_GLOBAL_BRIGHTNESS) {
    state->global_brightness--;
    state->global_brightness %= 8;
  } else if (state->config_var == CONFIG_VAR_ENABLED_VIZUALIZATIONS) {
    state->enabled_vizualizations[state->selected_viz] =
        !state->enabled_vizualizations[state->selected_viz];
  }
}

void reset_buffer() {
  memset(udp_buffer, 0, MAX_UDP_BUFFER_SIZE);
  buffer_pos = udp_buffer;
  udp_buffer[0] = 'B';
  udp_buffer[1] = 'D';
  buffer_pos += 2;
}

size_t buffer_size() { return buffer_pos - udp_buffer; }

void flush_buffer() {
  struct sockaddr_in serv_addr;
  int sockfd, i;

  sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sockfd == -1) {
    perror("Error opening socket");
    exit(1);
  }

  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT);
  if (inet_aton(IP_ADDRESS, &serv_addr.sin_addr) == 0) {
    perror("inet_aton() failed");
    exit(1);
  }

  if (sendto(sockfd,
             udp_buffer,
             buffer_size(),
             0,
             (struct sockaddr *)&serv_addr,
             sizeof(serv_addr)) == -1) {
    perror("sendto()");
  }
  close(sockfd);
  reset_buffer();
}

void write_byte(uint8_t b) {
  *buffer_pos = b;
  buffer_pos++;
}

void write_short(uint16_t s) {
  write_byte(s & 0xff);
  write_byte(s >> 8);
}

void set_pixel_value(
    uint8_t port, uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  if (buffer_size() + message_size > MAX_UDP_BUFFER_SIZE) {
    flush_buffer();
  }
  write_byte(port);
  write_short(index);
  write_byte(r);
  write_byte(g);
  write_byte(b);
  write_byte(w);
}

uint16_t animation_cycle = 0;

int viz_index = 0;

void switch_to_vizualization(vizualization_t *new_viz) {
  if (viz != NULL) {
    viz->deinitialize(viz);
  }
  viz = new_viz;
  led_info_t led_info = {.led_count = LED_COUNT};
  viz->initialize(viz, led_info);
}

void next_vizualization() {
  viz_index++;
  if (vizualizations[viz_index] == NULL) {
    viz_index = 0;
  }
  switch_to_vizualization(vizualizations[viz_index]);
}

void add_vizualization(app_state_t *state, vizualization_t *viz, bool enabled) {
  vizualizations[viz_index] = viz;
  state->enabled_vizualizations[viz_index] = enabled;
  viz_index++;
}

void app_state_enter_config_mode(app_state_t *state) {
  state->mode = APP_MODE_CONFIG;
  state->config_var = CONFIG_VAR_GLOBAL_BRIGHTNESS;
}

/*int main(int argc, char **argv) {
  while (true) {
    int key = keydown();
    if (config_viz.state.mode == APP_MODE_NORMAL) {
      if (key == 'j') {
        next_vizualization();
      } else if (key == 'k') {
        config_viz.state.mode = APP_MODE_CONFIG;
        switch_to_vizualization(&config_viz);
      }
    } else if (config_viz.state.mode == APP_MODE_CONFIG) {
      if (key == 'j') {
        config_increment_var(&config_viz.state);
      } else if (key == 'k') {
        next_vizualization();
        config_viz.state.mode = APP_MODE_NORMAL;
      }
    }
    usleep(16000);
  }
}*/

void handle_user_interaction(app_state_t *state,
                             app_vizualization_t *config_viz) {
  const Uint8 *keystate = SDL_GetKeyboardState(NULL);
  button_press_t red_press = detect_button_press(
      keystate[SDL_SCANCODE_J], &red_button, SDL_GetTicks());
  button_press_t green_press = detect_button_press(
      keystate[SDL_SCANCODE_K], &green_button, SDL_GetTicks());

  if (green_press == PRESS_LONG) {
    if (state->mode == APP_MODE_NORMAL) {
      state->mode = APP_MODE_CONFIG;
      switch_to_vizualization(config_viz);
    } else if (state->mode == APP_MODE_CONFIG) {
      state->mode = APP_MODE_NORMAL;
      next_vizualization();
    }
  }

  if (red_press == PRESS_LONG && state->mode == APP_MODE_CONFIG) {
    state->config_var++;
    state->config_var %= CONFIG_VAR_END;
  }

  if (red_press == PRESS_SINGLE) {
    if (state->mode == APP_MODE_CONFIG) {
      config_increment_var(state);
    } else {
      next_vizualization();
    }
  }

  if (green_press == PRESS_SINGLE) {
    if (state->mode == APP_MODE_CONFIG) {
      config_decrement_var(state);
    }
  }
}

int main(int argc, char *argv[]) {
  reset_buffer();
  rgb_t pixels[LED_COUNT];

  app_vizualization_t config_viz = {
      .viz =
          {
              .initialize = config_initialize,
              .deinitialize = config_deinitialize,
              .tick = config_tick,
              .get_pixel_values = config_get_pixel_values,
          },
      .state = {
          .mode = APP_MODE_NORMAL,
          .config_var = CONFIG_VAR_GLOBAL_BRIGHTNESS,
          .global_brightness = 7,
      }};

  add_vizualization(&config_viz.state, rainbow_vizualization(), false);
  add_vizualization(&config_viz.state, fire_vizualization(), true);
  add_vizualization(&config_viz.state, water_vizualization(), true);
  add_vizualization(&config_viz.state,
                    twinkle_vizualization((twinkle_config_t){.race = false}),
                    true);
  add_vizualization(&config_viz.state,
                    twinkle_vizualization((twinkle_config_t){.race = true}),
                    true);
  add_vizualization(&config_viz.state, alien_vizualization(), true);
  add_vizualization(&config_viz.state, sinusoidal_vizualization(), true);
  add_vizualization(&config_viz.state, lightning_vizualization(), false);
  add_vizualization(&config_viz.state, police_vizualization(), true);
  add_vizualization(&config_viz.state, lasers_vizualization(), true);
  add_vizualization(&config_viz.state, perlin_vizualization(), true);

  next_vizualization();

  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    fprintf(stderr, "Error initializing SDL: %s\n", SDL_GetError());
    return 1;
  }

  SDL_Window *window = SDL_CreateWindow("Multiple Key Press Detection",
                                        SDL_WINDOWPOS_UNDEFINED,
                                        SDL_WINDOWPOS_UNDEFINED,
                                        800,
                                        600,
                                        SDL_WINDOW_SHOWN);
  if (!window) {
    fprintf(stderr, "Error creating window: %s\n", SDL_GetError());
    SDL_Quit();
    return 1;
  }

  bool running = true;
  while (running) {
    SDL_Event event;

    // Poll for events
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        running = false;
      }
    }

    handle_user_interaction(&config_viz.state, &config_viz);

    viz->get_pixel_values(viz, pixels, LED_COUNT);
    uint8_t brightness = (config_viz.state.global_brightness * 32 + 31);
    for (uint16_t index = 0; index < LED_COUNT; index++) {
      set_pixel_value(0,
                      index,
                      pixels[index].r,
                      pixels[index].g,
                      pixels[index].b,
                      brightness);
    }
    viz->tick(viz, animation_cycle);
    animation_cycle++;
    flush_buffer();

    SDL_Delay(16); // Add a small delay to reduce CPU usage
  }

  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}