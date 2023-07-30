#include <math.h>
#include "color.h"
#include "viz.h"

typedef struct {
  vizualization_t viz;
  uint32_t animation_cycle;
} perlin_vizualization_t;

const uint16_t permutation_size = 256;
const uint8_t permutation[] = {
    151, 160, 137, 91,  90,  15,  131, 13,  201, 95,  96,  53,  194, 233, 7,
    225, 140, 36,  103, 30,  69,  142, 8,   99,  37,  240, 21,  10,  23,  190,
    6,   148, 247, 120, 234, 75,  0,   26,  197, 62,  94,  252, 219, 203, 117,
    35,  11,  32,  57,  177, 33,  88,  237, 149, 56,  87,  174, 20,  125, 136,
    171, 168, 68,  175, 74,  165, 71,  134, 139, 48,  27,  166, 77,  146, 158,
    231, 83,  111, 229, 122, 60,  211, 133, 230, 220, 105, 92,  41,  55,  46,
    245, 40,  244, 102, 143, 54,  65,  25,  63,  161, 1,   216, 80,  73,  209,
    76,  132, 187, 208, 89,  18,  169, 200, 196, 135, 130, 116, 188, 159, 86,
    164, 100, 109, 198, 173, 186, 3,   64,  52,  217, 226, 250, 124, 123, 5,
    202, 38,  147, 118, 126, 255, 82,  85,  212, 207, 206, 59,  227, 47,  16,
    58,  17,  182, 189, 28,  42,  223, 183, 170, 213, 119, 248, 152, 2,   44,
    154, 163, 70,  221, 153, 101, 155, 167, 43,  172, 9,   129, 22,  39,  253,
    19,  98,  108, 110, 79,  113, 224, 232, 178, 185, 112, 104, 218, 246, 97,
    228, 251, 34,  242, 193, 238, 210, 144, 12,  191, 179, 162, 241, 81,  51,
    145, 235, 249, 14,  239, 107, 49,  192, 214, 31,  181, 199, 106, 157, 184,
    84,  204, 176, 115, 121, 50,  45,  127, 4,   150, 254, 138, 236, 205, 93,
    222, 114, 67,  29,  24,  72,  243, 141, 128, 195, 78,  66,  215, 61,  156,
    180};

static float fade(float t) { return t * t * t * (t * (t * 6 - 15) + 10); }
static float lerp(float t, float a, float b) { return a + t * (b - a); }
static float grad_1d(int hash, float x) { return (hash & 1 ? x : -x); }
static float grad_2d(int hash, float x, float y) {
    int h = hash & 15;
    float u = h<8 ? x : y,
          v = h<4 ? y : h==12||h==14 ? x : 0;
    return ((h&1) == 0 ? u : -u) + ((h&2) == 0 ? v : -v);
}


float perlin_noise_1d(float x) {
  int xi = (int)x & (permutation_size - 1);
  float xf = x - (int)x;
  float u = fade(xf);
  int a = permutation[xi];
  int b = permutation[(xi + 1) & (permutation_size - 1)];
  return lerp(u, grad_1d(a, xf), grad_1d(b, xf - 1)) * 0.5 + 0.5;
}

float perlin_noise_2d(float x, float y) {
    int X = (int)floor(x) & (permutation_size -1);
    int Y = (int)floor(y) & (permutation_size -1);

    x -= floor(x);
    y -= floor(y);

    float u = fade(x);
    float v = fade(y);

    int A = permutation[X] + Y;
    int B = permutation[X+1] + Y;

    return lerp(v, lerp(u, grad_2d(permutation[A], x, y),
                        grad_2d(permutation[B], x - 1, y)),
                   lerp(u, grad_2d(permutation[A+1], x, y - 1),
                        grad_2d(permutation[B+1], x - 1, y - 1)));
}

void perlin_initialize(perlin_vizualization_t *viz, led_info_t led_info) {}
void perlin_deinitialize(perlin_vizualization_t *viz) {}

void perlin_get_pixel_values(perlin_vizualization_t *viz,
                             rgb_t *pixels,
                             uint16_t pixel_count) {
  float max = -500;
  float min = 500;
  for (uint16_t index = 0; index < pixel_count; index++) {
    float n = perlin_noise_2d((float)index / 32, (float)viz->animation_cycle / 32);
    float k = perlin_noise_2d((float)index / 16, (float)viz->animation_cycle / 32);
    pixels[index] = hsv_to_rgb(n+0.5, 1.0, pow(k+0.5, 4));
    max = fmax(max, n);
    max = fmax(max, k);

    min = fmin(min, n);
    min = fmin(min, k);
  }
}

void perlin_tick(perlin_vizualization_t *viz, uint32_t animation_cycle){
    viz->animation_cycle = animation_cycle;
};

REGISTER_VIZUALIZATION(perlin);