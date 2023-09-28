#include <inttypes.h>
#include <string.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include "color.h"
#include "led_strips.h"

static const char *TAG = "led_strips";

#define SR_SPI_TRANSACTION_COUNT 16
#define PORT_COUNT 8
#define LEDS_PER_PORT 55

#define SR_SRCLK_GPIO GPIO_NUM_4

#define SR_DATA_GPIO GPIO_NUM_15
#define SR_RCLK_GPIO GPIO_NUM_2

#define SR_SPI_HOST SPI2_HOST

rgb_t pixel_buffer[LEDS_PER_PORT][PORT_COUNT];

spi_device_handle_t spi;
uint8_t *dma_buffer = NULL;
size_t dma_buffer_size;

TaskHandle_t led_update_task_handle;

// Update LED strips at 60 FPS.
const TickType_t led_update_frequency = 60;

/*

Use in the future if need to accurately log gap between
SPI transactions.

typedef struct {
  uint64_t tx_start;
  uint64_t tx_end;
} tx_stats_t;

static tx_stats_t tx_stat1;
static tx_stats_t tx_stat2;
static uint8_t tx_stats_index = 0;

static void IRAM_ATTR tx_start_cb(spi_transaction_t *tx) {
  if (tx_stats_index == 0) {
    tx_stat1.tx_start = esp_timer_get_time();
  } else {
    tx_stat2.tx_start = esp_timer_get_time();
  }
}

static void IRAM_ATTR tx_end_cb(spi_transaction_t *tx) {
  if (tx_stats_index == 0) {
    tx_stat1.tx_end = esp_timer_get_time();
  } else {
    tx_stat2.tx_end = esp_timer_get_time();
  }
  tx_stats_index = (tx_stats_index + 1) % 2;
}*/

void led_strips_spi_bus_initialize() {
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .mosi_io_num = SR_RCLK_GPIO,
      .miso_io_num = SR_DATA_GPIO,
      .sclk_io_num = SR_SRCLK_GPIO,
      .flags = SPICOMMON_BUSFLAG_DUAL | SPICOMMON_BUSFLAG_MASTER,
      // TODO: Calculate max transfer size from parameters.
      .max_transfer_sz = 8192,
  };
  ret = spi_bus_initialize(SR_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = SPI_MASTER_FREQ_20M,
      .spics_io_num = -1,
      .queue_size = SR_SPI_TRANSACTION_COUNT,
      .flags = SPI_DEVICE_HALFDUPLEX,
      /*.pre_cb = tx_start_cb,
      .post_cb = tx_end_cb,*/
  };
  ret = spi_bus_add_device(SR_SPI_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);

  size_t max_spi_bytes;
  spi_bus_get_max_transaction_len(SR_SPI_HOST, &max_spi_bytes);
  ESP_LOGI(TAG, "Max SPI transaction length %d bytes", max_spi_bytes);
}

uint32_t led_strips_animation_cycle = 0;

void led_strips_animation_tick() {
  led_strips_animation_cycle++;
  for (uint16_t index = 0; index < LEDS_PER_PORT; index++) {
    rgb_t color = hsv_to_rgb(
        ((index + led_strips_animation_cycle) % 255) / 255.0f, 1.0, 1.0);
    if (index != 0 && index != 54) {
      color.r = 0;
      color.g = 0;
      color.b = 0;
    }
    for (uint8_t port = 0; port < PORT_COUNT; port++) {
      pixel_buffer[index][port] = color;
    }
  }
}

static const uint16_t morton_table_little_endian[256] = {
    0x0040, 0x0240, 0x0840, 0x0a40, 0x2040, 0x2240, 0x2840, 0x2a40, 0x8040,
    0x8240, 0x8840, 0x8a40, 0xa040, 0xa240, 0xa840, 0xaa40, 0x0042, 0x0242,
    0x0842, 0x0a42, 0x2042, 0x2242, 0x2842, 0x2a42, 0x8042, 0x8242, 0x8842,
    0x8a42, 0xa042, 0xa242, 0xa842, 0xaa42, 0x0048, 0x0248, 0x0848, 0x0a48,
    0x2048, 0x2248, 0x2848, 0x2a48, 0x8048, 0x8248, 0x8848, 0x8a48, 0xa048,
    0xa248, 0xa848, 0xaa48, 0x004a, 0x024a, 0x084a, 0x0a4a, 0x204a, 0x224a,
    0x284a, 0x2a4a, 0x804a, 0x824a, 0x884a, 0x8a4a, 0xa04a, 0xa24a, 0xa84a,
    0xaa4a, 0x0060, 0x0260, 0x0860, 0x0a60, 0x2060, 0x2260, 0x2860, 0x2a60,
    0x8060, 0x8260, 0x8860, 0x8a60, 0xa060, 0xa260, 0xa860, 0xaa60, 0x0062,
    0x0262, 0x0862, 0x0a62, 0x2062, 0x2262, 0x2862, 0x2a62, 0x8062, 0x8262,
    0x8862, 0x8a62, 0xa062, 0xa262, 0xa862, 0xaa62, 0x0068, 0x0268, 0x0868,
    0x0a68, 0x2068, 0x2268, 0x2868, 0x2a68, 0x8068, 0x8268, 0x8868, 0x8a68,
    0xa068, 0xa268, 0xa868, 0xaa68, 0x006a, 0x026a, 0x086a, 0x0a6a, 0x206a,
    0x226a, 0x286a, 0x2a6a, 0x806a, 0x826a, 0x886a, 0x8a6a, 0xa06a, 0xa26a,
    0xa86a, 0xaa6a, 0x00c0, 0x02c0, 0x08c0, 0x0ac0, 0x20c0, 0x22c0, 0x28c0,
    0x2ac0, 0x80c0, 0x82c0, 0x88c0, 0x8ac0, 0xa0c0, 0xa2c0, 0xa8c0, 0xaac0,
    0x00c2, 0x02c2, 0x08c2, 0x0ac2, 0x20c2, 0x22c2, 0x28c2, 0x2ac2, 0x80c2,
    0x82c2, 0x88c2, 0x8ac2, 0xa0c2, 0xa2c2, 0xa8c2, 0xaac2, 0x00c8, 0x02c8,
    0x08c8, 0x0ac8, 0x20c8, 0x22c8, 0x28c8, 0x2ac8, 0x80c8, 0x82c8, 0x88c8,
    0x8ac8, 0xa0c8, 0xa2c8, 0xa8c8, 0xaac8, 0x00ca, 0x02ca, 0x08ca, 0x0aca,
    0x20ca, 0x22ca, 0x28ca, 0x2aca, 0x80ca, 0x82ca, 0x88ca, 0x8aca, 0xa0ca,
    0xa2ca, 0xa8ca, 0xaaca, 0x00e0, 0x02e0, 0x08e0, 0x0ae0, 0x20e0, 0x22e0,
    0x28e0, 0x2ae0, 0x80e0, 0x82e0, 0x88e0, 0x8ae0, 0xa0e0, 0xa2e0, 0xa8e0,
    0xaae0, 0x00e2, 0x02e2, 0x08e2, 0x0ae2, 0x20e2, 0x22e2, 0x28e2, 0x2ae2,
    0x80e2, 0x82e2, 0x88e2, 0x8ae2, 0xa0e2, 0xa2e2, 0xa8e2, 0xaae2, 0x00e8,
    0x02e8, 0x08e8, 0x0ae8, 0x20e8, 0x22e8, 0x28e8, 0x2ae8, 0x80e8, 0x82e8,
    0x88e8, 0x8ae8, 0xa0e8, 0xa2e8, 0xa8e8, 0xaae8, 0x00ea, 0x02ea, 0x08ea,
    0x0aea, 0x20ea, 0x22ea, 0x28ea, 0x2aea, 0x80ea, 0x82ea, 0x88ea, 0x8aea,
    0xa0ea, 0xa2ea, 0xa8ea, 0xaaea};

#define INTERLEAVE(q, out) *(uint16_t *)(out) = morton_table_little_endian[q];

void init_dma_buffer(uint8_t *out, size_t count) {
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
  led_strips_spi_bus_initialize();
  // Set all colors to black.
  memset(pixel_buffer, 0, sizeof(rgb_t) * LEDS_PER_PORT * PORT_COUNT);
  led_strips_animation_tick();
  // Allocate DMA buffer.
  // Multiply by 2 for 2bit wide data line on SPI bus,
  // Multiply by 3 because 24bit color per LED,
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
// integers in little endian byte ordering: p3:p2:p1:p0, p4:p5:p6:p7.
#define UNPACK_PORTS_TO_DUAL_UINT32(f)                                         \
  const uint32_t u##f = pixel_buffer[i][3].f << 24 |                           \
                        pixel_buffer[i][2].f << 16 |                           \
                        pixel_buffer[i][1].f << 8 | pixel_buffer[i][0].f;      \
  const uint32_t l##f = pixel_buffer[i][4].f << 24 |                           \
                        pixel_buffer[i][5].f << 16 |                           \
                        pixel_buffer[i][6].f << 8 | pixel_buffer[i][7].f;

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

uint64_t total_time_spent = 0;
void led_strips_update() {

  uint64_t start = esp_timer_get_time();

  size_t tx_size = dma_buffer_size / SR_SPI_TRANSACTION_COUNT;

  uint8_t tx_count = 0;
  uint8_t *out = dma_buffer;
  uint8_t *tx_start = dma_buffer;

  esp_err_t ret;
  spi_transaction_t txns[SR_SPI_TRANSACTION_COUNT];

  for (uint16_t i = 0; i < LEDS_PER_PORT; i++) {
    // The follow is a series of macros that expand out into a large
    // "unrolled" loop that converts from color bytes to the SPI / LED
    // wire format as required for driving a 74HC595 shift register.
    //
    // Would love to be able to optimize this more
    UNPACK_PORTS_TO_DUAL_UINT32(r);
    INTERLEAVE_AND_OUTPUT_WIRE_FORMAT(r, out);
    UNPACK_PORTS_TO_DUAL_UINT32(g);
    INTERLEAVE_AND_OUTPUT_WIRE_FORMAT(g, out + 6 * 8);
    UNPACK_PORTS_TO_DUAL_UINT32(b);
    INTERLEAVE_AND_OUTPUT_WIRE_FORMAT(b, out + 2 * 6 * 8);

    out += 3 * 6 * 8;

    if (out - tx_start >= tx_size && tx_count != SR_SPI_TRANSACTION_COUNT - 1) {
      spi_transaction_t *tx = &txns[tx_count];
      memset(tx, 0, sizeof(spi_transaction_t));
      tx->length = 8 * (out - tx_start);
      tx->tx_buffer = (void *)tx_start;
      tx->flags = SPI_TRANS_MODE_DIO;

      ret = spi_device_queue_trans(spi, tx, portMAX_DELAY);
      ESP_ERROR_CHECK(ret);

      tx_count++;
      tx_start = out;
    }
  }
  // Send RCLK edge to output last byte.
  INTERLEAVE(0x00, out);
  out += 2;

  spi_transaction_t *tx = &txns[tx_count];
  memset(tx, 0, sizeof(spi_transaction_t));
  tx->length = 8 * (out - tx_start);
  tx->tx_buffer = (void *)tx_start;
  tx->flags = SPI_TRANS_MODE_DIO;

  ret = spi_device_queue_trans(spi, tx, portMAX_DELAY);
  ESP_ERROR_CHECK(ret);

  tx_count++;

  uint64_t end = esp_timer_get_time();

  total_time_spent += (end - start);

  if (led_strips_animation_cycle % 60 == 0) {
    ESP_LOGI(TAG, "Time spent mapping: %jdus", total_time_spent / 60);
    total_time_spent = 0;
  }

  if (out != dma_buffer + dma_buffer_size) {
    ESP_LOGE(TAG, "Invalid out pointer after dma buffer generation.");
  }

  // Wait for all the transactions to complete.
  spi_transaction_t *tx_result;
  for (uint8_t i = 0; i < tx_count; i++) {
    ret = spi_device_get_trans_result(spi, &tx_result, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
  }
}

static void led_strips_update_task(void *params) {
  TickType_t delay_ticks = (1000 / led_update_frequency) / portTICK_PERIOD_MS;
  TickType_t last_wake_time = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last_wake_time, delay_ticks);

    uint64_t animation_start = esp_timer_get_time();
    led_strips_animation_tick();
    uint64_t update_start = esp_timer_get_time();
    led_strips_update();
    uint64_t end = esp_timer_get_time();
    if (led_strips_animation_cycle % 60 == 0) {
      ESP_LOGI(TAG,
               "Time taken to animate: %jdus update: %jdus",
               update_start - animation_start,
               end - update_start);
    }
  }
}

void led_strips_start_update_task() {
  ESP_LOGI(TAG, "Starting update task...");
  xTaskCreatePinnedToCore(led_strips_update_task,
                          "led_strips_update",
                          4096,
                          NULL,
                          4 /* Priority */,
                          &led_update_task_handle,
                          1 /* CPU affinity */);
}

/* Debugging: output Qc and Qd are dead, also Qa starts at port index 2. */

// Qa red
// Qb green
// Qc no output
// Qd no output
// Qe blue
// Qf yellow
// Qg cyan
// Qh magneta