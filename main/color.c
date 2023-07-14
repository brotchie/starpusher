#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "color.h"

#define FIRE_PALETTE_SIZE 256
#define FIRE_NOISE_SIZE 256

rgb_t hsv_to_rgb(float h, float s, float v) {
  float c = v * s;
  float x = c * (1.0f - fabsf(fmodf(h * 6.0f, 2.0f) - 1.0f));
  float m = v - c;

  float r, g, b;
  if (h >= 0.0f && h < 1.0f / 6.0f) {
    r = c, g = x, b = 0.0f;
  } else if (h < 1.0f / 3.0f) {
    r = x, g = c, b = 0.0f;
  } else if (h < 0.5f) {
    r = 0.0f, g = c, b = x;
  } else if (h < 2.0f / 3.0f) {
    r = 0.0f, g = x, b = c;
  } else if (h < 5.0f / 6.0f) {
    r = x, g = 0.0f, b = c;
  } else {
    r = c, g = 0.0f, b = x;
  }

  rgb_t result;
  result.r = (uint8_t)((r + m) * 255.0f);
  result.g = (uint8_t)((g + m) * 255.0f);
  result.b = (uint8_t)((b + m) * 255.0f);

  return result;
}

rgb_t fire_palette[FIRE_PALETTE_SIZE] = {0};
uint8_t fire_noise[FIRE_NOISE_SIZE] = {0};

void fire_palette_initialize() {
  int i;

  // black to red
  for (i = 0; i < 64; ++i) {
    fire_palette[i].r = i * 4;
    fire_palette[i].g = 0;
    fire_palette[i].b = 0;
  }

  // red to orange
  for (i = 64; i < 128; ++i) {
    fire_palette[i].r = 255;
    fire_palette[i].g = (i - 64) * 4;
    fire_palette[i].b = 0;
  }

  // orange to yellow
  for (i = 128; i < 192; ++i) {
    fire_palette[i].r = 255;
    fire_palette[i].g = 255;
    fire_palette[i].b = (i - 128) * 4;
  }

  // yellow to white
  for (i = 192; i < 256; ++i) {
    fire_palette[i].r = 255;
    fire_palette[i].g = 255;
    fire_palette[i].b = 255;
  }

  for (int x = 0; x < FIRE_NOISE_SIZE; x++) {
    fire_noise[x] = rand() % FIRE_PALETTE_SIZE;
  }
}

void fire_palette_update_noise() {
  memmove(fire_noise, fire_noise + 1, FIRE_NOISE_SIZE - 1);
  fire_noise[FIRE_NOISE_SIZE - 1] = rand() % FIRE_PALETTE_SIZE;
}

rgb_t fire_palette_get_pixel_value(uint16_t index) {
  uint16_t noise_index = (index + (rand() % 3 - 1)) % FIRE_NOISE_SIZE;
  uint16_t color_index = fire_noise[noise_index];
  return fire_palette[color_index];
}

#define WATER_PALETTE_SIZE 256
#define WATER_NOISE_SIZE 256

// Initialize a water palette
rgb_t water_palette[WATER_PALETTE_SIZE] = {0}; // Stores RGB values
uint8_t water_noise[WATER_NOISE_SIZE] = {0};   // The noise array

void water_palette_initialize() {
  for (int x = 0; x < WATER_PALETTE_SIZE / 2; x++) {
    water_palette[x].r = 0; // Keep red component low for water
    water_palette[x].g = (x < 128) ? x << 1 : 255 - ((x - 128) << 1);
    water_palette[x].b =
        (x < 64) ? x << 3 : 255; // Keep blue high for water effect

    water_palette[WATER_PALETTE_SIZE - 1 - x].r =
        0; // Keep red component low for water
    water_palette[WATER_PALETTE_SIZE - 1 - x].g =
        (x < 128) ? x << 1 : 255 - ((x - 128) << 1);
    water_palette[WATER_PALETTE_SIZE - 1 - x].b =
        (x < 64) ? x << 3 : 255; // Keep blue high for water effect
  }
}

rgb_t water_palette_get_pixel_value(uint16_t index) {
  if (rand() % 32 == 0) {
    rgb_t white = {.r = 255, .g = 255, .b = 255};
    return white;
  }
  return water_palette[index % WATER_PALETTE_SIZE];
}

#define TWINKLE_BUFFER_SIZE 40

rgb_t twinkle_buffer[TWINKLE_BUFFER_SIZE] = {0};

rgb_t twinkle_buffer_get_pixel_value(uint16_t index) {
  return twinkle_buffer[index % TWINKLE_BUFFER_SIZE];
}

void twinkle_buffer_tick() {
  if (rand() % 10 == 0) {
    twinkle_buffer[rand() % TWINKLE_BUFFER_SIZE] =
        hsv_to_rgb((double)(rand() % 256) / 256.0, 1.0, 1.0);
  }
  for (uint16_t i = 0; i < TWINKLE_BUFFER_SIZE; i++) {
    twinkle_buffer[i].r *= 0.95;
    twinkle_buffer[i].g *= 0.95;
    twinkle_buffer[i].b *= 0.95;
  }
}

#define ALIEN_BUFFER_SIZE 40

rgb_t alien_buffer[ALIEN_BUFFER_SIZE] = {0};

rgb_t alien_buffer_get_pixel_value(uint16_t index) {
  if (rand() % 120 == 0) {
    rgb_t white = {.r = 255, .g = 255, .b = 0};
    return white;
  }
  return alien_buffer[index % ALIEN_BUFFER_SIZE];
}

void alien_buffer_tick(uint16_t animation_cycle) {
  if (rand() % 10 == 0) {
    alien_buffer[rand() % ALIEN_BUFFER_SIZE].g = 255;
  }
  for (uint16_t i = 0; i < ALIEN_BUFFER_SIZE; i++) {
    alien_buffer[i].g *= 0.95;
  }
  if (animation_cycle % 30 == 0) {
    rgb_t first = alien_buffer[0];
    memmove(alien_buffer, alien_buffer + 1, ALIEN_BUFFER_SIZE - 1);
    alien_buffer[ALIEN_BUFFER_SIZE - 1] = first;
  }
}
