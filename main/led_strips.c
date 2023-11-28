#include <inttypes.h>
#include <string.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include "color.h"
#include "led_strips.h"

static const char *TAG = "led_strips";

//#define BUILT_IN_ANIMATIONS
//#define PRINT_TIMING_DEBUG

#define PORT_COUNT 8

// 3 triangles with 55 LEDs on each side.
#define LEDS_PER_PORT 9 * 55

#define SPI_MAX_TRANSFER_SIZE 71282

// Drive SPI clock at 11Mhz. At 13Mhz it starts to glitch
// out a LED strip of length 495. At 10MHz is doesn't fit
// in the 13.3ms time bound for 60FPS.
#define SR_SRCLK_FREQUENCY SPI_MASTER_FREQ_11M

#define SR_SRCLK_GPIO GPIO_NUM_4

#define SR_SER0_GPIO GPIO_NUM_14
#define SR_SER1_GPIO GPIO_NUM_15
#define SR_RCLK_GPIO GPIO_NUM_2
// Unfortunately the SPI driver won't work in QUAD mode
// without mapping the data3 pin through GPIO (maybe
// there's a way around this?). We simply map the 4th
// SPI line to the CFG pin on the WT32-ETH0, which I
// don't think is used for anything.
#define SR_ZERO_GPIO GPIO_NUM_32

#define SR_SPI_HOST SPI2_HOST

static rgb_t pixel_buffer[PORT_COUNT][LEDS_PER_PORT];
static SemaphoreHandle_t pixel_buffer_mutex;

static spi_device_handle_t spi;
static uint8_t *dma_buffer = NULL;
static size_t dma_buffer_size;

static TaskHandle_t led_update_task_handle;

static TickType_t led_update_frequency = 60;

static uint32_t led_strips_animation_cycle = 0;

static uint64_t total_time_spent = 0;

void led_strips_spi_bus_initialize() {
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .data0_io_num = SR_RCLK_GPIO,
      .data1_io_num = SR_SER0_GPIO,
      .data2_io_num = SR_SER1_GPIO,
      .data3_io_num = SR_ZERO_GPIO,
      .sclk_io_num = SR_SRCLK_GPIO,
      .flags = SPICOMMON_BUSFLAG_QUAD | SPICOMMON_BUSFLAG_MASTER,
      .max_transfer_sz = SPI_MAX_TRANSFER_SIZE,
      // Pin all SPI ISR handling on CPU core 1.
      .isr_cpu_id = INTR_CPU_ID_1,
  };
  ret = spi_bus_initialize(SR_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = SR_SRCLK_FREQUENCY,
      .spics_io_num = -1,
      .queue_size = 1,
      // Don't need to receive SPI.
      .flags = SPI_DEVICE_HALFDUPLEX,
  };
  ret = spi_bus_add_device(SR_SPI_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);
}

void led_strips_animation_tick() {
  for (uint16_t index = 0; index < LEDS_PER_PORT; index++) {
    rgb_t color = hsv_to_rgb(
        ((index + led_strips_animation_cycle) % 255) / 255.0f, 1.0, 1.0);
    for (uint8_t port = 0; port < PORT_COUNT; port++) {
      pixel_buffer[port][index] = color;
    }
  }
}

// Here we've pre-calculated the interleaving of all possible bytes into the
// packed format for quad-SPI DMA. This trades off memory (512 bytes) to save
// a bunch of shift and masking operations.
//
// These values also include bits for the shift register clock signal on SPI
// DATA0 and the empty last channel of quad-SPI on SPI DATA3.
static const uint16_t interleave_table_little_endian[256] = {
    0x0010, 0x0410, 0x4010, 0x4410, 0x0014, 0x0414, 0x4014, 0x4414, 0x0050,
    0x0450, 0x4050, 0x4450, 0x0054, 0x0454, 0x4054, 0x4454, 0x0210, 0x0610,
    0x4210, 0x4610, 0x0214, 0x0614, 0x4214, 0x4614, 0x0250, 0x0650, 0x4250,
    0x4650, 0x0254, 0x0654, 0x4254, 0x4654, 0x2010, 0x2410, 0x6010, 0x6410,
    0x2014, 0x2414, 0x6014, 0x6414, 0x2050, 0x2450, 0x6050, 0x6450, 0x2054,
    0x2454, 0x6054, 0x6454, 0x2210, 0x2610, 0x6210, 0x6610, 0x2214, 0x2614,
    0x6214, 0x6614, 0x2250, 0x2650, 0x6250, 0x6650, 0x2254, 0x2654, 0x6254,
    0x6654, 0x0012, 0x0412, 0x4012, 0x4412, 0x0016, 0x0416, 0x4016, 0x4416,
    0x0052, 0x0452, 0x4052, 0x4452, 0x0056, 0x0456, 0x4056, 0x4456, 0x0212,
    0x0612, 0x4212, 0x4612, 0x0216, 0x0616, 0x4216, 0x4616, 0x0252, 0x0652,
    0x4252, 0x4652, 0x0256, 0x0656, 0x4256, 0x4656, 0x2012, 0x2412, 0x6012,
    0x6412, 0x2016, 0x2416, 0x6016, 0x6416, 0x2052, 0x2452, 0x6052, 0x6452,
    0x2056, 0x2456, 0x6056, 0x6456, 0x2212, 0x2612, 0x6212, 0x6612, 0x2216,
    0x2616, 0x6216, 0x6616, 0x2252, 0x2652, 0x6252, 0x6652, 0x2256, 0x2656,
    0x6256, 0x6656, 0x0030, 0x0430, 0x4030, 0x4430, 0x0034, 0x0434, 0x4034,
    0x4434, 0x0070, 0x0470, 0x4070, 0x4470, 0x0074, 0x0474, 0x4074, 0x4474,
    0x0230, 0x0630, 0x4230, 0x4630, 0x0234, 0x0634, 0x4234, 0x4634, 0x0270,
    0x0670, 0x4270, 0x4670, 0x0274, 0x0674, 0x4274, 0x4674, 0x2030, 0x2430,
    0x6030, 0x6430, 0x2034, 0x2434, 0x6034, 0x6434, 0x2070, 0x2470, 0x6070,
    0x6470, 0x2074, 0x2474, 0x6074, 0x6474, 0x2230, 0x2630, 0x6230, 0x6630,
    0x2234, 0x2634, 0x6234, 0x6634, 0x2270, 0x2670, 0x6270, 0x6670, 0x2274,
    0x2674, 0x6274, 0x6674, 0x0032, 0x0432, 0x4032, 0x4432, 0x0036, 0x0436,
    0x4036, 0x4436, 0x0072, 0x0472, 0x4072, 0x4472, 0x0076, 0x0476, 0x4076,
    0x4476, 0x0232, 0x0632, 0x4232, 0x4632, 0x0236, 0x0636, 0x4236, 0x4636,
    0x0272, 0x0672, 0x4272, 0x4672, 0x0276, 0x0676, 0x4276, 0x4676, 0x2032,
    0x2432, 0x6032, 0x6432, 0x2036, 0x2436, 0x6036, 0x6436, 0x2072, 0x2472,
    0x6072, 0x6472, 0x2076, 0x2476, 0x6076, 0x6476, 0x2232, 0x2632, 0x6232,
    0x6632, 0x2236, 0x2636, 0x6236, 0x6636, 0x2272, 0x2672, 0x6272, 0x6672,
    0x2276, 0x2676, 0x6276, 0x6676};

#define INTERLEAVE(q, out)                                                     \
  *(uint16_t *)(out) = interleave_table_little_endian[q];

void init_dma_buffer(uint8_t *out, size_t count) {
  // Only the middle bit of each physical bit on the wire changes. The
  // first and last bit are always 1 and 0 respectively.
  for (size_t i = 0; i < 3 * count; i++) {
    INTERLEAVE(0xff, out);
    out += 2;
    INTERLEAVE(0x00, out);
    out += 2;
    INTERLEAVE(0x00, out);
    out += 2;
  }
}

void led_strips_initialize() {
  pixel_buffer_mutex = xSemaphoreCreateMutex();
  led_strips_spi_bus_initialize();
  // Initially set all colors to black.
  if (xSemaphoreTake(pixel_buffer_mutex, portMAX_DELAY) == pdTRUE) {
    memset(pixel_buffer, 0, sizeof(rgb_t) * LEDS_PER_PORT * PORT_COUNT);
    xSemaphoreGive(pixel_buffer_mutex);
  }
#ifdef LED_STRIP_ANIMATION
  led_strips_animation_tick();
#endif
  // Allocate DMA buffer.
  // Multiply by 2 because we're using quad-SPI with 4 bits to SR0, 4 bits
  //     to SR1, 4 bits for RCLK, and 4 bits of zeros for unused SPI DATA3.
  // Multiply by 3 because 24 bit color per LED,
  // Multiply by 3 because 1 logical bit maps to 3 physical bits on LEDs,
  // Add 2 because we need to clock out the final pixels value and then
  // clock out zeros from the shift register outputs.
  dma_buffer_size = 2 * 3 * 3 * LEDS_PER_PORT * PORT_COUNT + 2;
  ESP_LOGI(TAG, "DMA Buffer Size: %d", dma_buffer_size);
  dma_buffer =
      heap_caps_malloc(dma_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  if (dma_buffer == NULL) {
    ESP_LOGE(TAG, "Failed to allocate DMA memory for LED strips");
    return;
  }
  init_dma_buffer(dma_buffer, LEDS_PER_PORT * PORT_COUNT);
}

// Unpacks the colors channel f (r, g, b) of 8 ports into two unsigned 32bit
// integers in little endian byte ordering: p7:p6:p4:p4, p3:p2:p1:p0, this
// ensures that the bits are in the correct positions so that output signals
// align with Ports 1-8.
//
// This is much faster (twice as fast) as unpacking into a 8 x uint8_t array. We
// then do operations directly on 2 x 32bit values rather than 8 x 8bit values.
#define UNPACK_PORTS_TO_DUAL_UINT32(f)                                         \
  const uint32_t u##f = pixel_buffer[7][i].f << 24 |                           \
                        pixel_buffer[6][i].f << 16 |                           \
                        pixel_buffer[5][i].f << 8 | pixel_buffer[4][i].f;      \
  const uint32_t l##f = pixel_buffer[3][i].f << 24 |                           \
                        pixel_buffer[2][i].f << 16 |                           \
                        pixel_buffer[1][i].f << 8 | pixel_buffer[0][i].f;

#define CALCULATE_BYTE(f, b, out)                                              \
  const uint32_t u##f##_##b = u##f & (0x80808080 >> b);                        \
  const uint32_t l##f##_##b = l##f & (0x80808080 >> b);                        \
  const uint8_t d##f##b =                                                      \
      (u##f##_##b >> (0 + 7 - b)) | (u##f##_##b >> (8 + 6 - b)) |              \
      (u##f##_##b >> (16 + 5 - b)) | (u##f##_##b >> (24 + 4 - b)) |            \
      (3 - b < 0 ? (l##f##_##b << (b - 3)) : (l##f##_##b >> (3 - b))) |        \
      (l##f##_##b >> (8 + 2 - b)) | (l##f##_##b >> (16 + 1 - b)) |             \
      (l##f##_##b >> (24 - b));                                                \
  INTERLEAVE(d##f##b, (out) + 2 + 6 * b);

#define INTERLEAVE_AND_OUTPUT_WIRE_FORMAT(f, out)                              \
  CALCULATE_BYTE(f, 0, out);                                                   \
  CALCULATE_BYTE(f, 1, out);                                                   \
  CALCULATE_BYTE(f, 2, out);                                                   \
  CALCULATE_BYTE(f, 3, out);                                                   \
  CALCULATE_BYTE(f, 4, out);                                                   \
  CALCULATE_BYTE(f, 5, out);                                                   \
  CALCULATE_BYTE(f, 6, out);                                                   \
  CALCULATE_BYTE(f, 7, out);

void led_strips_update() {
  uint64_t start = esp_timer_get_time();
  uint8_t *out = dma_buffer;

  if (xSemaphoreTake(pixel_buffer_mutex, portMAX_DELAY) == pdTRUE) {
    for (uint16_t i = 0; i < LEDS_PER_PORT; i++) {
      // The follow is a series of macros that expand out into a large
      // "unrolled" loop that converts from color bytes to the SPI / LED
      // wire format as required for driving a 74HC595 shift register.
      //
      // SKC6809MINI-HV-4P take color in RGB order (different from WS2812B).
      UNPACK_PORTS_TO_DUAL_UINT32(r);
      INTERLEAVE_AND_OUTPUT_WIRE_FORMAT(r, out);
      UNPACK_PORTS_TO_DUAL_UINT32(g);
      INTERLEAVE_AND_OUTPUT_WIRE_FORMAT(g, out + 6 * 8);
      UNPACK_PORTS_TO_DUAL_UINT32(b);
      INTERLEAVE_AND_OUTPUT_WIRE_FORMAT(b, out + 2 * 6 * 8);

      out += 3 * 6 * 8;
    }
    xSemaphoreGive(pixel_buffer_mutex);
  }
  // Make sure we send a final RCLK edge to clock last bit into pixels.
  INTERLEAVE(0x00, out);
  out += 2;

  total_time_spent += (esp_timer_get_time() - start);

#ifdef PRINT_TIMING_DEBUG
  if (led_strips_animation_cycle % 60 == 0) {
    ESP_LOGI(TAG, "Time spent mapping: %jdus", total_time_spent / 60);
    total_time_spent = 0;
  }
#endif

  // Total data written should be the same length as the DMA buffer.
  if (out != dma_buffer + dma_buffer_size) {
    ESP_LOGE(TAG, "Invalid out pointer after dma buffer generation.");
  }

  spi_transaction_t tx;
  memset(&tx, 0, sizeof(spi_transaction_t));
  // SPI transaction length is in bits!
  tx.length = 8 * dma_buffer_size;
  tx.tx_buffer = (void *)dma_buffer;
  // Drive 4x data lines.
  tx.flags = SPI_TRANS_MODE_QIO;

  // Disable interrupts while we're sending SPI so nothing gets interrupted.
  portDISABLE_INTERRUPTS();
  esp_err_t ret = spi_device_polling_transmit(spi, &tx);
  portENABLE_INTERRUPTS();

  ESP_ERROR_CHECK(ret);
}

static void led_strips_update_task(void *params) {
  led_strips_initialize();

  TickType_t delay_ticks = (1000 / led_update_frequency) / portTICK_PERIOD_MS;
  TickType_t last_wake_time = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last_wake_time, delay_ticks);

#ifdef PRINT_TIMING_DEBUG
    uint64_t animation_start = esp_timer_get_time();
#endif

#ifdef BUILT_IN_ANIMATIONS
    led_strips_animation_tick();
#endif

#ifdef PRINT_TIMING_DEBUG
    uint64_t update_start = esp_timer_get_time();
#endif

    led_strips_update();

#ifdef PRINT_TIMING_DEBUG
    uint64_t end = esp_timer_get_time();
    if (led_strips_animation_cycle % 60 == 0) {
      ESP_LOGI(TAG,
               "Time taken to animate: %jdus update: %jdus",
               update_start - animation_start,
               end - update_start);
    }
#endif
  }
}

void led_strips_bulk_update(uint8_t port,
                            uint16_t start_index,
                            uint16_t count,
                            rgb_t *data) {
  if (port < 1 || port > PORT_COUNT) {
    ESP_LOGE(TAG, "Invalid port %d, must be in range [1, 8]", port);
  }
  if (start_index >= LEDS_PER_PORT || start_index + count > LEDS_PER_PORT) {
    ESP_LOGE(TAG,
             "Invalid LED range [%d, %d], must be in range [0, %d]",
             start_index,
             start_index + count - 1,
             LEDS_PER_PORT - 1);
  }
  if (xSemaphoreTake(pixel_buffer_mutex, portMAX_DELAY) == pdTRUE) {
    // We've defined the UDP packet format so it can be directly memcpy'd from
    // the packet buffer!
    memcpy(&pixel_buffer[port - 1][start_index], data, 3 * count);
    xSemaphoreGive(pixel_buffer_mutex);
  }
}

void led_strips_start_update_task() {
  ESP_LOGI(TAG, "Starting LED update task...");
  xTaskCreatePinnedToCore(led_strips_update_task,
                          "led_strips_update",
                          4096,
                          NULL,
                          20 /* Priority */,
                          &led_update_task_handle,
                          1 /* CPU affinity */);
}
