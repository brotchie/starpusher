#include <inttypes.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "buffered_led_strips.h"

static const char *TAG = "buffered_led_strips";

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
#define PER_STRIP_LED_COUNT 420

// 2Mhz is enough to get 60 FPS for 4x 420 LED strips.
#define DEFAULT_CLOCK_SPEED_HZ 2000000

typedef struct __attribute__((__packed__)) {
  uint32_t clock_speed_hz;
} spi_config_message_t;

typedef uint8_t output_select_state;

typedef struct {
  spi_device_handle_t *spi_device;
  output_select_state output_select;
  uint16_t led_count;
  uint8_t *dma_buffer;
  size_t dma_buffer_size;
} buffered_led_strip_t;

spi_device_handle_t spi2;
spi_device_handle_t spi3;

buffered_led_strip_t strips[LED_STRIP_COUNT];

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
  };
  ret = spi_bus_add_device(host_id, &devcfg, spi_dev);
  ESP_ERROR_CHECK(ret);

  // Drive the DATA and CLK pins as hard as possible.
  ret = gpio_set_drive_capability(mosi_io_num, GPIO_DRIVE_CAP_3);
  ESP_ERROR_CHECK(ret);
  ret = gpio_set_drive_capability(sclk_io_num, GPIO_DRIVE_CAP_3);
  ESP_ERROR_CHECK(ret);
}

void spi_device_deinitialize(spi_host_device_t host_id,
                             spi_device_handle_t *spi_dev) {
  esp_err_t ret;
  ret = spi_bus_remove_device(*spi_dev);
  ESP_ERROR_CHECK(ret);
  ret = spi_bus_free(host_id);
  ESP_ERROR_CHECK(ret);
}

uint8_t calculate_end_frame_count(uint16_t led_count) {
  return (led_count / 16) + 1;
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
void buffered_led_strip_reset_buffer(buffered_led_strip_t *strip) {
  if (strip->dma_buffer == NULL) {
    return;
  }

  // Completely reset the buffer to zeros.
  memset(strip->dma_buffer, 0, strip->dma_buffer_size);

  // Sets the first byte of each LED frame to its default state (0xe0).
  for (uint32_t i = 0; i < strip->led_count; i++) {
    strip->dma_buffer[FRAME_SIZE * (1 + i)] = 0xe0;
  }

  // Sets all bits in the ends frames to one.
  uint16_t end_frame_count = calculate_end_frame_count(strip->led_count);
  size_t end_frame_start = FRAME_SIZE * (1 + strip->led_count);
  size_t end_frame_size = FRAME_SIZE * end_frame_count;
  memset(strip->dma_buffer + end_frame_start, 0xff, end_frame_size);
}

/* Initializes an led strip, allocating its DMA buffer.

Note the passed in strip MUST have its led_count field set.
*/
void buffered_led_strip_initialize(buffered_led_strip_t *strip) {
  if (strip->dma_buffer) {
    // TODO(brotchie): Handle already initialized.
    return;
  }

  // Allocate this LED strip's buffer in DMA-capable byte-accessible heap.
  uint16_t end_frame_count = calculate_end_frame_count(strip->led_count);
  size_t dma_buffer_size =
      FRAME_SIZE * (1 + strip->led_count + end_frame_count);
  strip->dma_buffer_size = dma_buffer_size;
  strip->dma_buffer =
      heap_caps_malloc(dma_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  if (strip->dma_buffer == NULL) {
    ESP_LOGE(TAG, "Allocating memory for LED strip failed");
    return;
  }
  buffered_led_strip_reset_buffer(strip);
}

void buffered_led_strip_deinitialize(buffered_led_strip_t *strip) {
  if (strip->dma_buffer != NULL) {
    heap_caps_free(strip->dma_buffer);
    strip->dma_buffer = NULL;
  }
}

void buffered_led_strips_initialize_default() {
  buffered_led_strips_initialize(DEFAULT_CLOCK_SPEED_HZ);
}

void buffered_led_strips_initialize(uint32_t clock_speed_hz) {
  ESP_LOGI(TAG,
           "Initializing %d x %d strips of LEDs. Driving LEDs at %luMhz.",
           LED_STRIP_COUNT,
           PER_STRIP_LED_COUNT,
           clock_speed_hz / 1000000);

  output_select_initialize();

  spi_device_initialize(
      SPI2_HOST, &spi2, clock_speed_hz, SPI2_MOSI_GPIO, SPI2_CLK_GPIO);
  spi_device_initialize(
      SPI3_HOST, &spi3, clock_speed_hz, SPI3_MOSI_GPIO, SPI3_CLK_GPIO);

  // LED0 port
  strips[0].spi_device = &spi2;
  strips[0].output_select = OUTPUT_SELECT_LED0_LED2;
  strips[0].led_count = PER_STRIP_LED_COUNT;

  // LED1 port
  strips[1].spi_device = &spi2;
  strips[1].output_select = OUTPUT_SELECT_LED1_LED3;
  strips[1].led_count = PER_STRIP_LED_COUNT;

  // LED2 port
  strips[2].spi_device = &spi3;
  strips[2].output_select = OUTPUT_SELECT_LED0_LED2;
  strips[2].led_count = PER_STRIP_LED_COUNT;

  // LED3 port
  strips[3].spi_device = &spi3;
  strips[3].output_select = OUTPUT_SELECT_LED1_LED3;
  strips[3].led_count = PER_STRIP_LED_COUNT;

  for (uint8_t i = 0; i < LED_STRIP_COUNT; i++) {
    buffered_led_strip_initialize(&strips[i]);
  }
}

void buffered_led_strips_deinitialize() {
  for (uint8_t i = 0; i < LED_STRIP_COUNT; i++) {
    buffered_led_strip_deinitialize(&strips[i]);
  }
  spi_device_deinitialize(SPI2_HOST, &spi2);
  spi_device_deinitialize(SPI3_HOST, &spi3);
}

void buffered_led_strip_set_pixel_value(buffered_led_strip_t *strip,
                                        uint16_t index,
                                        uint8_t r,
                                        uint8_t g,
                                        uint8_t b,
                                        uint8_t w) {
  if (strip->dma_buffer == NULL) {
    ESP_LOGE(TAG, "LED strip DMA buffer not initialized");
    return;
  }
  if (index >= strip->led_count) {
    ESP_LOGE(TAG, "Invalid index on LED strip");
    return;
  }
  strip->dma_buffer[FRAME_SIZE * (1 + index)] = 0xe0 | (w >> 3);
  strip->dma_buffer[FRAME_SIZE * (1 + index) + 1] = b;
  strip->dma_buffer[FRAME_SIZE * (1 + index) + 2] = g;
  strip->dma_buffer[FRAME_SIZE * (1 + index) + 3] = r;
}

inline void buffered_led_strips_set_pixel_value(
    uint8_t port, uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  if (port >= LED_STRIP_COUNT) {
    // TODO(brotchie) Invalid port
    return;
  }
  buffered_led_strip_set_pixel_value(&strips[port], index, r, g, b, w);
}

void buffered_led_strips_update_for_output_select(
    output_select_state output_select,
    buffered_led_strip_t *strip1,
    buffered_led_strip_t *strip2) {
  if (strip1->spi_device == strip2->spi_device) {
    // TODO(brotchie) Strips should be on separate SPI buses.
    return;
  }
  if (strip1->output_select != output_select ||
      strip2->output_select != output_select) {
    // TODO(brotchie) Strips should both be for this output select.
    return;
  }

  output_select_set(output_select);

  // Get DMA transactions ready for each SPI bus.
  spi_transaction_t tx1;
  memset(&tx1, 0, sizeof(tx1));
  // SPI DMA transactions expect their length in bits!
  tx1.length = 8 * strip1->dma_buffer_size;
  tx1.tx_buffer = (void *)strip1->dma_buffer;

  spi_transaction_t tx2;
  memset(&tx2, 0, sizeof(tx2));
  tx2.length = 8 * strip2->dma_buffer_size;
  tx2.tx_buffer = (void *)strip2->dma_buffer;

  // Start both SPI output transactions at once so they can run in parallel.
  esp_err_t ret;
  ret = spi_device_polling_start(*strip1->spi_device, &tx1, portMAX_DELAY);
  ESP_ERROR_CHECK(ret);
  ret = spi_device_polling_start(*strip2->spi_device, &tx2, portMAX_DELAY);
  ESP_ERROR_CHECK(ret);

  // Block on transactions to complete.
  ret = spi_device_polling_end(*strip1->spi_device, portMAX_DELAY);
  ESP_ERROR_CHECK(ret);
  ret = spi_device_polling_end(*strip2->spi_device, portMAX_DELAY);
  ESP_ERROR_CHECK(ret);
}

uint8_t update_ticks;

void buffered_led_strips_update() {
  update_ticks++;
  uint64_t start_micros = esp_timer_get_time();
  buffered_led_strips_update_for_output_select(
      OUTPUT_SELECT_LED0_LED2, &strips[0], &strips[2]);
  buffered_led_strips_update_for_output_select(
      OUTPUT_SELECT_LED1_LED3, &strips[1], &strips[3]);
  uint64_t end_micros = esp_timer_get_time();
  double duration_millis = (end_micros - start_micros) / 1000.0;
  if (update_ticks % 60 == 0) {
    ESP_LOGI(TAG,
             "Output for %d strips took %.2fms, achievable FPS %.2f",
             LED_STRIP_COUNT,
             duration_millis,
             1000 / duration_millis);
  }
}

void buffered_led_strips_reset() {
  for (uint8_t i = 0; i < LED_STRIP_COUNT; i++) {
    buffered_led_strip_reset_buffer(&strips[i]);
  }
  buffered_led_strips_update();
}