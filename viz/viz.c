#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>

#include "color.h"
#include "viz.h"

#define IP_ADDRESS "10.1.1.100"
#define PORT 6868
#define MAX_UDP_BUFFER_SIZE 1472
#define LED_COUNT 128

const size_t message_size = 7;
uint8_t udp_buffer[MAX_UDP_BUFFER_SIZE];
uint8_t *buffer_pos = udp_buffer;

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

int main(int argc, char **argv) {
    reset_buffer();
    rgb_t pixels[LED_COUNT];

    vizualization_t* vizualizations[] = {
        //rainbow_vizualization(),
        //fire_vizualization(),
        //water_vizualization(),
        //twinkle_vizualization((twinkle_config_t){.race = false}),
        //twinkle_vizualization((twinkle_config_t){.race = true}),
        //alien_vizualization(),
        //sinusoidal_vizualization(),
        //lightning_vizualization(),
        //police_vizualization(),
        //lasers_vizualization(),
        perlin_vizualization(),
    };
    led_info_t led_info = {.led_count=LED_COUNT};
    uint8_t viz_count = sizeof(vizualizations) / sizeof(vizualization_t*);
    uint8_t viz_index = 0;
    printf("Initialized %ld\n", viz_count);

    vizualization_t *viz = vizualizations[viz_index];
    viz->initialize(viz, led_info);
    while (true) {
        viz->get_pixel_values(viz, pixels, LED_COUNT);
        for (uint16_t index = 0; index < LED_COUNT; index++) {
            set_pixel_value(0, index, pixels[index].r, pixels[index].g, pixels[index].b, 150);
        }
        viz->tick(viz, animation_cycle);
        animation_cycle++;
        flush_buffer();
        if (animation_cycle % 300 == 0) {
            viz_index++;
            viz_index %= viz_count;
            viz->deinitialize(viz);
            viz = vizualizations[viz_index];
            viz->initialize(viz, led_info);
        }
        usleep(16000);
    }
}