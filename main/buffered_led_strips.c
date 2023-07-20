#include <inttypes.h>
#include <string.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include "buffered_led_strips.h"
#include "color.h"

static const char *TAG = "buffered_led_strips";

#define UPDATE_TASK_PRIORITY 4

// We have the TCP/IP stack and packet processing pinned to
// CPU0, so pinning the LED update task to CPU1 gives us
// an empty core!
#define UPDATE_TASK_CPU_AFFINITY 1

// GPIOs controlling the demuxers selecting between (LED0, LED2)
// and (LED1, LED3) as output ports.
#define OUTPUT_SELECT_GPIO GPIO_NUM_5
#define OUTPUT_SELECT_LED0_LED2 0
#define OUTPUT_SELECT_LED1_LED3 1

// Output GPIOs for SPI2.
#define SPI2_MOSI_GPIO GPIO_NUM_4
#define SPI2_CLK_GPIO GPIO_NUM_2

// Output GPIOs for SPI3.
#define SPI3_MOSI_GPIO GPIO_NUM_15
#define SPI3_CLK_GPIO GPIO_NUM_14

// LED strips configuration.
#define FRAME_SIZE 4
#define LED_STRIP_COUNT 4
#define PER_STRIP_LED_COUNT 128

// 2Mhz is enough to get 60 FPS for 4x 420 LED strips.
#define DEFAULT_APA102_CLOCK_SPEED_HZ 2000000
#define REQUIRED_WS2812B_CLOCK_SPEED_HZ 3000000

// Aim for 60 FPS.
const TickType_t update_frequency = 60;

TaskHandle_t update_task_handle;

typedef enum {
  LED_DEVICE_APA102 = 0,
  LED_DEVICE_WS2812B = 1,
} led_device_type_t;

typedef uint8_t output_select_state;

typedef struct {
  led_device_type_t device_type;
  spi_device_handle_t *spi_device;
  output_select_state output_select;
  uint16_t led_count;
  uint8_t *raw_buffer;
  size_t raw_buffer_size;
} buffered_led_strip_t;

spi_device_handle_t spi2;
spi_device_handle_t spi3;

uint8_t *spi2_dma_buffer = NULL;
uint8_t *spi3_dma_buffer = NULL;

// Mutex to manage access to the LED strip buffers. This enables
// both the networking and LED update tasks to safely access
// buffer contents.
SemaphoreHandle_t strip_buffers_mutex;

// This is a singleton mutex that's used to acquire exclusive ownership of
// all SPI buses. This prevents SPI reconfigures (e.g. changing SPI clock speed)
// from conflicting with LED updates.
SemaphoreHandle_t spi_mutex;

buffered_led_strip_t strips[LED_STRIP_COUNT];

// If true, then output a rainbow chase pattern.
bool vizualizations_state = false;

void buffered_led_strip_reset_buffer(
    buffered_led_strip_t *strip, uint8_t r, uint8_t g, uint8_t b, uint8_t w);

/* Initializes the OUT_SEL pin to the LED strip demultiplexers. */
void output_select_initialize() {
  esp_err_t ret = gpio_set_direction(OUTPUT_SELECT_GPIO, GPIO_MODE_OUTPUT);
  ESP_ERROR_CHECK(ret);
}

/* Selects which LED strip ports connect to which SPI bus:

    OUTPUT_SELECT_LED0_LED2 (spi2 = LED0, spi3 = LED2)
    OUTPUT_SELECT_LED1_LED3 (spi2 = LED1, spi3 = LED3)
*/
void output_select_set(output_select_state state) {
  esp_err_t ret = gpio_set_level(OUTPUT_SELECT_GPIO, state);
  ESP_ERROR_CHECK(ret);
}

// Initializes a SPI host with a single device.
//
// The ESP32 has two SPI ports available for general use:
// SPI2 and SPI3. Beacuse we're using them for output-only
// we only need to configure two GPIOs (SCLK and MOSI).
void spi_device_initialize(spi_host_device_t host_id,
                           spi_device_handle_t *spi_dev,
                           int clock_speed_hz,
                           int mosi_io_num,
                           int sclk_io_num) {
  esp_err_t ret;
  spi_bus_config_t buscfg = {.miso_io_num = -1,
                             .mosi_io_num = mosi_io_num,
                             .sclk_io_num = sclk_io_num,
                             .quadwp_io_num = -1,
                             .quadhd_io_num = -1};
  ret = spi_bus_initialize(host_id, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = clock_speed_hz,
      .spics_io_num = -1,
      .queue_size = 7,
      .command_bits = 0,
      .address_bits = 0,
      .dummy_bits = 0,
  };
  ret = spi_bus_add_device(host_id, &devcfg, spi_dev);
  ESP_ERROR_CHECK(ret);

  // Up the drive strength of the DATA and CLK pins as hard as possible.
  ret = gpio_set_drive_capability(mosi_io_num, GPIO_DRIVE_CAP_3);
  ESP_ERROR_CHECK(ret);
  ret = gpio_set_drive_capability(sclk_io_num, GPIO_DRIVE_CAP_3);
  ESP_ERROR_CHECK(ret);
}

// Cleans up a previously initialized SPI device. Used to reset
// the SPI bus on SPI clock speed changes.
void spi_device_deinitialize(spi_host_device_t host_id,
                             spi_device_handle_t *spi_dev) {
  esp_err_t ret;
  ret = spi_bus_remove_device(*spi_dev);
  ESP_ERROR_CHECK(ret);
  ret = spi_bus_free(host_id);
  ESP_ERROR_CHECK(ret);
}

// The APA102 protocol requires at least one end-frame of
// all ones for every 16 LEDs on the strip. This function
// calculates the required end frames and then adds one
// extra for luck.
uint8_t calculate_apa102_end_frame_count(uint16_t led_count) {
  return (led_count / 16) + 1;
}

/* Initializes an led strip, allocating its DMA buffer.

Note the passed in strip MUST have its led_count field set.
*/
void buffered_led_strip_initialize(buffered_led_strip_t *strip) {
  if (strip->raw_buffer) {
    // TODO(brotchie): Handle already initialized.
    return;
  }

  if (strip->device_type == LED_DEVICE_APA102) {
    uint16_t end_frame_count =
        calculate_apa102_end_frame_count(strip->led_count);
    size_t raw_buffer_size =
        FRAME_SIZE * (1 + strip->led_count + end_frame_count);
    strip->raw_buffer_size = raw_buffer_size;
  } else if (strip->device_type == LED_DEVICE_WS2812B) {
    // We're abusing the SPI data pin to clock out the WS2812B wire
    // format. One logical bit sent to the WS2812B strip is actually
    // three physical SPI bits.
    //
    // Each LED on the strip takes 24bits of data ordered g, r, b.
    // 3 x 24 is 8*9 so it will always be evenly divisible by 8.
    strip->raw_buffer_size = (3 * 24 * strip->led_count) / 8;
  }

  strip->raw_buffer = heap_caps_malloc(strip->raw_buffer_size, MALLOC_CAP_8BIT);
  if (strip->raw_buffer == NULL) {
    ESP_LOGE(TAG, "Allocating memory for LED strip failed");
    return;
  }

  buffered_led_strip_reset_buffer(strip, 0, 0, 0, 0);
}

void buffered_led_strip_deinitialize(buffered_led_strip_t *strip) {
  if (strip->raw_buffer != NULL) {
    heap_caps_free(strip->raw_buffer);
    strip->raw_buffer = NULL;
  }
}

// Safely copies the LED strip's current pixel buffer to a target DMA buffer.
// (Safe because it acquires a lock on the pixel buffer's mutex).
//
// Returns the length, in bytes, of the LED update output.
size_t buffered_led_strip_safe_copy_to_dma_buffer(buffered_led_strip_t *strip,
                                                  uint8_t *dma_buffer) {
  if (xSemaphoreTake(strip_buffers_mutex, portMAX_DELAY) == pdTRUE) {
    memcpy(dma_buffer, strip->raw_buffer, strip->raw_buffer_size);
    xSemaphoreGive(strip_buffers_mutex);
  }
  return strip->raw_buffer_size;
}

void buffered_led_strips_initialize(uint32_t clock_speed_hz) {
  fire_palette_initialize();
  water_palette_initialize();

  // LED0 port
  strips[0].device_type = LED_DEVICE_WS2812B;
  strips[0].output_select = OUTPUT_SELECT_LED0_LED2;
  strips[0].led_count = PER_STRIP_LED_COUNT;

  // LED1 port
  strips[1].device_type = LED_DEVICE_WS2812B;
  strips[1].output_select = OUTPUT_SELECT_LED1_LED3;
  strips[1].led_count = PER_STRIP_LED_COUNT;

  // LED2 port
  strips[2].device_type = LED_DEVICE_WS2812B;
  strips[2].output_select = OUTPUT_SELECT_LED0_LED2;
  strips[2].led_count = PER_STRIP_LED_COUNT;

  // LED3 port
  strips[3].device_type = LED_DEVICE_WS2812B;
  strips[3].output_select = OUTPUT_SELECT_LED1_LED3;
  strips[3].led_count = PER_STRIP_LED_COUNT;

  // If any of the attached strips are WS2812B, we have to use a 3Mhz clock
  // speed.
  for (uint8_t i = 0; i < LED_STRIP_COUNT; i++) {
    if (strips[i].device_type == LED_DEVICE_WS2812B) {
      ESP_LOGI(TAG,
               "Using required WS2812B clock speed: %dMhz",
               REQUIRED_WS2812B_CLOCK_SPEED_HZ / 1000000);
      clock_speed_hz = REQUIRED_WS2812B_CLOCK_SPEED_HZ;
      break;
    }
  }

  ESP_LOGI(TAG,
           "Initializing %d x %d strips of LEDs. Driving LEDs at %luMhz.",
           LED_STRIP_COUNT,
           PER_STRIP_LED_COUNT,
           clock_speed_hz / 1000000);

  strip_buffers_mutex = xSemaphoreCreateMutex();
  if (spi_mutex == NULL) {
    spi_mutex = xSemaphoreCreateMutex();
  }

  output_select_initialize();

  spi_device_initialize(
      SPI2_HOST, &spi2, clock_speed_hz, SPI2_MOSI_GPIO, SPI2_CLK_GPIO);
  strips[0].spi_device = &spi2;
  strips[1].spi_device = &spi2;
  spi_device_initialize(
      SPI3_HOST, &spi3, clock_speed_hz, SPI3_MOSI_GPIO, SPI3_CLK_GPIO);
  strips[2].spi_device = &spi3;
  strips[3].spi_device = &spi3;

  for (uint8_t i = 0; i < LED_STRIP_COUNT; i++) {
    buffered_led_strip_initialize(&strips[i]);
  }

  // Allocate SPI buffers in memory with DMA capability.
  spi2_dma_buffer = heap_caps_malloc(strips[0].raw_buffer_size,
                                     MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  if (spi2_dma_buffer == NULL) {
    ESP_LOGE(TAG, "Failed to allocate DMA memory for SPI2");
    return;
  }
  spi3_dma_buffer = heap_caps_malloc(strips[2].raw_buffer_size,
                                     MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  if (spi3_dma_buffer == NULL) {
    ESP_LOGE(TAG, "Failed to allocate DMA memory for SPI3");
    return;
  }
}

// Initializes all the machinery to drive LED strips (using default SPI speed).
void buffered_led_strips_initialize_default() {
  buffered_led_strips_initialize(DEFAULT_APA102_CLOCK_SPEED_HZ);
}

// Cleans up all of the machinery to drive LED strips.
void buffered_led_strips_deinitialize() {
  vSemaphoreDelete(strip_buffers_mutex);

  for (uint8_t i = 0; i < LED_STRIP_COUNT; i++) {
    buffered_led_strip_deinitialize(&strips[i]);
  }
  spi_device_deinitialize(SPI2_HOST, &spi2);
  spi_device_deinitialize(SPI3_HOST, &spi3);
  if (spi2_dma_buffer) {
    heap_caps_free(spi2_dma_buffer);
    spi2_dma_buffer = NULL;
  }
  if (spi3_dma_buffer) {
    heap_caps_free(spi3_dma_buffer);
    spi3_dma_buffer = NULL;
  }
}

// Safely reconfigures SPI2 and SPI3 with a new SPI speed.
void buffered_led_strips_safe_reconfigure(uint32_t clock_speed_hz) {
  if (xSemaphoreTake(spi_mutex, portMAX_DELAY) == pdTRUE) {
    buffered_led_strips_deinitialize();
    buffered_led_strips_initialize(clock_speed_hz);
    xSemaphoreGive(spi_mutex);
  }
}

// Acquires an exclusive lock on the LED strip's buffer mutex.
//
// It's too expensive to acquire and release a mutex for every single
// set_pixel call, so the UDP packet handling thread grabs a mutex
// while it's processing a LED update message.
//
// If the LED update thread happens to wake up while UDP packet handling
// is still happening, then it will immediately output LED updates
// to the strips once packet handling is done.
uint8_t buffered_led_strips_acquire_buffers_mutex() {
  if (xSemaphoreTake(strip_buffers_mutex, portMAX_DELAY) == pdTRUE) {
    return 1;
  } else {
    return 0;
  }
}

void buffered_led_strips_release_buffers_mutex() {
  xSemaphoreGive(strip_buffers_mutex);
}

// Sets the r, g, b and brightness value of a LED at index
// on a given LED strip. This is an internal function,
// the external equivalent which takes a port index as the first
// arg is buffered_led_strips_set_pixel_value.
void buffered_led_strip_set_pixel_value(buffered_led_strip_t *strip,
                                        uint16_t index,
                                        uint8_t r,
                                        uint8_t g,
                                        uint8_t b,
                                        uint8_t w) {
  if (strip->raw_buffer == NULL) {
    ESP_LOGE(TAG, "LED strip DMA buffer not initialized");
    return;
  }
  if (index >= strip->led_count) {
    ESP_LOGE(TAG, "Invalid index on LED strip");
    return;
  }
  if (strip->device_type == LED_DEVICE_APA102) {
    strip->raw_buffer[FRAME_SIZE * (1 + index)] = 0xe0 | (w >> 3);
    strip->raw_buffer[FRAME_SIZE * (1 + index) + 1] = b;
    strip->raw_buffer[FRAME_SIZE * (1 + index) + 2] = g;
    strip->raw_buffer[FRAME_SIZE * (1 + index) + 3] = r;
  } else if (strip->device_type == LED_DEVICE_WS2812B) {
    uint32_t start_index = 3 * 24 * index;
    uint8_t scaled_g = g * ((float)w / 255.0);
    uint8_t scaled_r = r * ((float)w / 255.0);
    uint8_t scaled_b = b * ((float)w / 255.0);
    for (int i = 0; i < 8; i++) {
      uint8_t g_bit = (scaled_g >> (7 - i)) & 0x01;
      uint8_t r_bit = (scaled_r >> (7 - i)) & 0x01;
      uint8_t b_bit = (scaled_b >> (7 - i)) & 0x01;

      uint32_t g_byte_index = (start_index + i * 3 + 1) / 8;
      uint32_t g_bit_index = (start_index + i * 3 + 1) % 8;

      uint32_t r_byte_index = (start_index + (i + 8) * 3 + 1) / 8;
      uint32_t r_bit_index = (start_index + (i + 8) * 3 + 1) % 8;

      uint32_t b_byte_index = (start_index + (i + 16) * 3 + 1) / 8;
      uint32_t b_bit_index = (start_index + (i + 16) * 3 + 1) % 8;

      if (g_bit) {
        strip->raw_buffer[g_byte_index] |= (1 << (7 - g_bit_index));
      } else {
        strip->raw_buffer[g_byte_index] &= ~(1 << (7 - g_bit_index));
      }

      if (r_bit) {
        strip->raw_buffer[r_byte_index] |= (1 << (7 - r_bit_index));
      } else {
        strip->raw_buffer[r_byte_index] &= ~(1 << (7 - r_bit_index));
      }

      if (b_bit) {
        strip->raw_buffer[b_byte_index] |= (1 << (7 - b_bit_index));
      } else {
        strip->raw_buffer[b_byte_index] &= ~(1 << (7 - b_bit_index));
      }
    }
  }
}

/* Reset's a LED strip's buffer to turn all LEDs off.

The DMA buffer contains the raw wire format for signalling
to apa102 LEDs.

    0x00000000 - 4 byte start frame of zeros
    0xewbbggrr - 4 byte led frame always starting with two ones (0xe0)
                 with the lower 6 bits of this first byte being
                 pwm brightness. The following bytes are bgr colors.
    0xffffffff - One of more end frames at least one additional end
                 frame per 16 LEDs is required to ensure all data is
                 clocked through the strip.
*/
void buffered_led_strip_reset_buffer(
    buffered_led_strip_t *strip, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  if (strip->raw_buffer == NULL) {
    return;
  }

  // Completely reset the buffer to zeros.
  memset(strip->raw_buffer, 0, strip->raw_buffer_size);

  if (strip->device_type == LED_DEVICE_APA102) {
    // For APA102 all bits in the ends frames to one.
    uint16_t end_frame_count =
        calculate_apa102_end_frame_count(strip->led_count);
    size_t end_frame_start = FRAME_SIZE * (1 + strip->led_count);
    size_t end_frame_size = FRAME_SIZE * end_frame_count;
    memset(strip->raw_buffer + end_frame_start, 0xff, end_frame_size);
  } else if (strip->device_type == LED_DEVICE_WS2812B) {
    // For WS2812B set the first bit of every 3 bits to one (zero).
    for (uint32_t i = 0; i < 8 * strip->raw_buffer_size; i += 3) {
      uint32_t byte_index = i / 8;
      uint8_t bit_index = i % 8;
      strip->raw_buffer[byte_index] |= (1 << (7 - bit_index));
    }
  }
  // Sets the first byte of each LED frame to its reset value.
  for (uint32_t i = 0; i < strip->led_count; i++) {
    buffered_led_strip_set_pixel_value(strip, i, r, g, b, w);
  }
}

// Sets the r, g, b, and brightness value of a LED at index on the
// given output port.
//
// Note, you must first acquire an exclusive lock on the LED strip buffers
// before calling this function (using
// buffered_led_strips_acquire_buffers_mutex).
inline void buffered_led_strips_set_pixel_value(
    uint8_t port, uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  if (port >= LED_STRIP_COUNT) {
    // TODO(brotchie) Invalid port
    return;
  }
  buffered_led_strip_set_pixel_value(&strips[port], index, r, g, b, w);
}

// Updates two LED strips for a given output select value.
//
// The Starpusher board has four headers for LED strips (labeled
// LED0, LED1, LED2, LED3). Between the ESP32 and these headers
// are two demultiplexers with a shared output select.
//
// Settings the output select to low will route SPI2 and SPI3
// to ports LED0 and LED2 respectively. Setting it high will route
// to LED1 and LED3.
//
// This allows us to efficiently drive four LED strips at 60FPS
// using only two SPI buses.
void buffered_led_strips_update_for_output_select(
    output_select_state output_select,
    buffered_led_strip_t *strip1,
    buffered_led_strip_t *strip2) {
  if (strip1->spi_device != &spi2) {
    ESP_LOGE(TAG, "strip1 must be on SPI2");
    return;
  }
  if (strip2->spi_device != &spi3) {
    ESP_LOGE(TAG, "strip2 must be on SPI3");
    return;
  }
  if (strip1->output_select != output_select ||
      strip2->output_select != output_select) {
    ESP_LOGE(TAG, "Both strips must be for the same output select");
    return;
  }

  output_select_set(output_select);

  // Get DMA transactions ready for each SPI bus.
  spi_transaction_t tx1;
  memset(&tx1, 0, sizeof(tx1));
  // SPI DMA transactions expect their length in bits!
  size_t tx1_length =
      buffered_led_strip_safe_copy_to_dma_buffer(strip1, spi2_dma_buffer);
  tx1.length = 8 * tx1_length;
  tx1.tx_buffer = (void *)spi2_dma_buffer;

  spi_transaction_t tx2;
  memset(&tx2, 0, sizeof(tx2));
  size_t tx2_length =
      buffered_led_strip_safe_copy_to_dma_buffer(strip2, spi3_dma_buffer);
  tx2.length = 8 * tx2_length;
  tx2.tx_buffer = (void *)spi3_dma_buffer;

  // Start both SPI output transactions at once so they can run in parallel.
  esp_err_t ret;
  ret = spi_device_polling_start(*strip1->spi_device, &tx1, portMAX_DELAY);
  ESP_ERROR_CHECK(ret);
  ret = spi_device_polling_start(*strip2->spi_device, &tx2, portMAX_DELAY);
  ESP_ERROR_CHECK(ret);

  // Block on transactions to complete (it's ok to do this since the LED
  // update thread is exclusively running on CPU1).
  ret = spi_device_polling_end(*strip1->spi_device, portMAX_DELAY);
  ESP_ERROR_CHECK(ret);
  ret = spi_device_polling_end(*strip2->spi_device, portMAX_DELAY);
  ESP_ERROR_CHECK(ret);
}

uint16_t update_ticks;

void buffered_led_strips_update() {
  update_ticks++;
  uint64_t start_micros = esp_timer_get_time();
  buffered_led_strips_update_for_output_select(
      OUTPUT_SELECT_LED0_LED2, &strips[0], &strips[2]);
  // buffered_led_strips_update_for_output_select(
  //   OUTPUT_SELECT_LED1_LED3, &strips[1], &strips[3]);
  uint64_t end_micros = esp_timer_get_time();
  double duration_millis = (end_micros - start_micros) / 1000.0;
  if (update_ticks % 600 == 0) {
    ESP_LOGI(TAG,
             "Output for %d strips took %.2fms, achievable FPS %.2f",
             LED_STRIP_COUNT,
             duration_millis,
             1000 / duration_millis);
  }
}

// Resets the buffers of all LED strips.
//
// Note, you must first acquire an exclusive lock on the LED strip buffers
// before calling this function (using
// buffered_led_strips_acquire_buffers_mutex).
void buffered_led_strips_reset(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  for (uint8_t i = 0; i < LED_STRIP_COUNT; i++) {
    buffered_led_strip_reset_buffer(&strips[i], r, g, b, w);
  }
}

void buffered_led_strips_set_vizualiations(bool state) {
  vizualizations_state = state;
}

uint16_t rainbow_chase_cycle = 0;
uint16_t animation_cycle = 0;
uint8_t rainbow_chase_cycle_length = 128;

#define VISUALIZATION_RAINBOW 0
#define VISUALIZATION_FIRE 1
#define VISUALIZATION_WATER 2
#define VISUALIZATION_TWINKLE 3
#define VISUALIZATION_TWINKLE_RACE 4
#define VISUALIZATION_ALIEN 5
#define VISUALIZATION_PROB_WEIGHT 6
#define VISUALIZATION_COLOR_LIGHTNING 7

uint8_t visualization_type = VISUALIZATION_COLOR_LIGHTNING;
uint8_t visualization_brightness = 150;

void buffered_led_strips_update_vizualizations() {
  uint16_t led_count = strips[0].led_count;
  animation_cycle++;
  if (animation_cycle % 600 == 0) {
    visualization_type++;
    visualization_type %= 8;
  }
  if (visualization_type == 0) {
    rainbow_chase_cycle++;
    for (uint8_t strip = 0; strip < LED_STRIP_COUNT; strip++) {
      for (uint16_t index = 0; index < led_count; index++) {
        double hue = (double)((index + rainbow_chase_cycle) %
                              rainbow_chase_cycle_length) /
                     (double)rainbow_chase_cycle_length;
        rgb_t rgb = hsv_to_rgb(hue, 1.0, 1.0);
        buffered_led_strip_set_pixel_value(&strips[strip],
                                           index,
                                           rgb.r,
                                           rgb.g,
                                           rgb.b,
                                           visualization_brightness);
      }
    }
  } else if (visualization_type == 1) {
    for (uint8_t strip = 0; strip < LED_STRIP_COUNT; strip++) {
      for (uint16_t index = 0; index < led_count; index++) {
        rgb_t rgb = fire_palette_get_pixel_value(index);
        buffered_led_strip_set_pixel_value(&strips[strip],
                                           index,
                                           rgb.r,
                                           rgb.g,
                                           rgb.b,
                                           visualization_brightness);
      }
    }
    if (animation_cycle % 10 == 0) {
      fire_palette_update_noise();
    }
  } else if (visualization_type == 2) {
    for (uint8_t strip = 0; strip < LED_STRIP_COUNT; strip++) {
      for (uint16_t index = 0; index < led_count; index++) {
        rgb_t rgb = water_palette_get_pixel_value(animation_cycle + index);
        buffered_led_strip_set_pixel_value(&strips[strip],
                                           index,
                                           rgb.r,
                                           rgb.g,
                                           rgb.b,
                                           visualization_brightness);
      }
    }
  } else if (visualization_type == 3) {
    for (uint8_t strip = 0; strip < LED_STRIP_COUNT; strip++) {
      for (uint16_t index = 0; index < led_count; index++) {
        rgb_t rgb = twinkle_buffer_get_pixel_value(index);
        buffered_led_strip_set_pixel_value(&strips[strip],
                                           index,
                                           rgb.r,
                                           rgb.g,
                                           rgb.b,
                                           visualization_brightness);
      }
    }
    twinkle_buffer_tick();
  } else if (visualization_type == 4) {
    for (uint8_t strip = 0; strip < LED_STRIP_COUNT; strip++) {
      for (uint16_t index = 0; index < led_count; index++) {
        rgb_t rgb = twinkle_buffer_get_pixel_value(index + animation_cycle);
        buffered_led_strip_set_pixel_value(&strips[strip],
                                           index,
                                           rgb.r,
                                           rgb.g,
                                           rgb.b,
                                           visualization_brightness);
      }
    }
    twinkle_buffer_tick();
  } else if (visualization_type == 5) {
    for (uint8_t strip = 0; strip < LED_STRIP_COUNT; strip++) {
      for (uint16_t index = 0; index < led_count; index++) {
        rgb_t rgb = alien_buffer_get_pixel_value(index);
        buffered_led_strip_set_pixel_value(&strips[strip],
                                           index,
                                           rgb.r,
                                           rgb.g,
                                           rgb.b,
                                           visualization_brightness);
      }
    }
    alien_buffer_tick(animation_cycle);
  } else if (visualization_type == 6) {
    for (uint8_t strip = 0; strip < LED_STRIP_COUNT; strip++) {
      for (uint16_t index = 0; index < led_count; index++) {
        rgb_t rgb = prob_weight_buffer_get_pixel_value(index);
        buffered_led_strip_set_pixel_value(&strips[strip],
                                           index,
                                           rgb.r,
                                           rgb.g,
                                           rgb.b,
                                           visualization_brightness);
      }
    }
    prob_weight_buffer_tick(animation_cycle);
  } else if (visualization_type == 7) {
    for (uint8_t strip = 0; strip < LED_STRIP_COUNT; strip++) {
      for (uint16_t index = 0; index < led_count; index++) {
        rgb_t rgb = color_lightning_get_pixel_value(index);
        buffered_led_strip_set_pixel_value(&strips[strip],
                                           index,
                                           rgb.r,
                                           rgb.g,
                                           rgb.b,
                                           visualization_brightness);
      }
    }
    color_lightning_tick(animation_cycle);
  }
}

static void buffered_led_strips_update_task(void *params) {
  // We could possibly just hot-loop here without delay, since
  // we only acquire a lock on LED strip buffers for a short time
  // while we memcpy them into the SPI DMA buffers, and we're
  // the only task running on CPU1. Just to be safe, however, we to
  // output a steady 60Hz output.
  ESP_LOGI(TAG, "Targeting LED updates at ~%ldHz: %ldms update period", update_frequency, 1000/update_frequency);
  TickType_t last_wake_time;
  TickType_t delay_ticks = (1000 / update_frequency) / portTICK_PERIOD_MS;
  last_wake_time = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last_wake_time, delay_ticks);

    if (vizualizations_state) {
      if (buffered_led_strips_acquire_buffers_mutex()) {
        buffered_led_strips_update_vizualizations();
        buffered_led_strips_release_buffers_mutex();
      }
    }
    if (xSemaphoreTake(spi_mutex, portMAX_DELAY) == pdTRUE) {
      buffered_led_strips_update();
      xSemaphoreGive(spi_mutex);
    }
  }
}

void buffered_led_strips_start_update_task() {
  ESP_LOGI(TAG, "Starting update task...");
  xTaskCreatePinnedToCore(buffered_led_strips_update_task,
                          "buffered_led_strips_update",
                          4096,
                          NULL,
                          UPDATE_TASK_PRIORITY,
                          &update_task_handle,
                          UPDATE_TASK_CPU_AFFINITY);
}