#ifndef UDP_PROTOCOL_H
#define UDP_PROTOCOL_H

#include <inttypes.h>
#include <sys/types.h>

void process_udp_packet(uint8_t *packet, ssize_t size);

#endif