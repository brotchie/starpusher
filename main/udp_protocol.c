#include <inttypes.h>
#include <string.h>
#include <sys/types.h>

#include <esp_log.h>
#include <esp_timer.h>

#include "led_strips.h"
#include "udp_protocol.h"

static const char *TAG = "udp_protocol";

// A pixel update message starts with a 7-byte preamble:
// The magic string "*P" followed by port, index, and count.
// The remainder of the packet is count * 24bit RGB values.
// Count is at most 488 to fit within the standard UDP
// maximum packet size.
typedef struct __attribute__((__packed__)) {
  uint8_t port;
  uint16_t index;
  uint16_t count;
} led_update_message_t;

void process_led_packet(uint8_t *packet, ssize_t size) {
  led_update_message_t *message = (led_update_message_t *)packet;
  rgb_t *colors = (rgb_t *)(packet + sizeof(led_update_message_t));
  led_strips_bulk_update(message->port, message->index, message->count, colors);
}

void process_udp_packet(uint8_t *packet, ssize_t size) {
  if (size < sizeof(led_update_message_t)) {
    // No point doing anything if there's less bytes than the 7-byte preamble.
    return;
  }
  // LED update packets start with the magic string "*P" (Starpusher).
  if (packet[0] == '*' && packet[1] == 'P') {
    process_led_packet(packet + 2, size - 2);
  }
}