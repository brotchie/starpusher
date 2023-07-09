#include <inttypes.h>
#include <sys/types.h>

#include "esp_log.h"

#include "buffered_led_strips.h"

static const char *TAG = "udp_protocol";

typedef struct __attribute__((__packed__)) {
  uint8_t port;
  uint16_t index;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t w;
} led_update_message_t;

typedef struct __attribute__((__packed__)) {
  uint32_t clock_speed_hz;
} spi_config_message_t;

void process_led_packet(uint8_t *packet, ssize_t size) {
  if (size % sizeof(led_update_message_t) != 0) {
    ESP_LOGI(TAG,
             "Invalid packet size %d should be a multiple of %d",
             size,
             sizeof(led_update_message_t));
    return;
  }
  uint16_t packet_count = size / sizeof(led_update_message_t);
  led_update_message_t *messages = (led_update_message_t *)packet;
  for (uint16_t i = 0; i < packet_count; i++) {
    led_update_message_t *message = &messages[i];
    if (message->port == 0xff && message->index == 0xffff) {
      buffered_led_strips_update();
    } else {
      buffered_led_strips_set_pixel_value(message->port,
                                          message->index,
                                          message->r,
                                          message->g,
                                          message->b,
                                          message->w);
    }
  }
}

void process_spi_config_packet(uint8_t *packet, ssize_t size) {
  spi_config_message_t *message = (spi_config_message_t *)packet;
  buffered_led_strips_deinitialize();
  buffered_led_strips_initialize(message->clock_speed_hz);
}

void process_udp_packet(uint8_t *packet, ssize_t size) {
  if (size <= 3) {
    return;
  }
  // LED update packets start with the magic string "BD",
  // SPI config packets start with the magirc string "SPI".
  if (packet[0] == 'B' && packet[1] == 'D') {
    process_led_packet(packet + 2, size - 2);
  } else if (packet[0] == 'S' && packet[1] == 'P' && packet[2] == 'I') {
    process_spi_config_packet(packet + 3, size - 3);
  } else {
    return;
  }
}
