from typing import List, Tuple

import struct
import socket
import math

MAX_PIXELS_PER_UDP_PACKET = 488

GAMMA_CORRECTION = [
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    2,
    2,
    2,
    2,
    2,
    2,
    2,
    2,
    3,
    3,
    3,
    3,
    3,
    3,
    3,
    4,
    4,
    4,
    4,
    4,
    5,
    5,
    5,
    5,
    6,
    6,
    6,
    6,
    7,
    7,
    7,
    7,
    8,
    8,
    8,
    9,
    9,
    9,
    10,
    10,
    10,
    11,
    11,
    11,
    12,
    12,
    13,
    13,
    13,
    14,
    14,
    15,
    15,
    16,
    16,
    17,
    17,
    18,
    18,
    19,
    19,
    20,
    20,
    21,
    21,
    22,
    22,
    23,
    24,
    24,
    25,
    25,
    26,
    27,
    27,
    28,
    29,
    29,
    30,
    31,
    32,
    32,
    33,
    34,
    35,
    35,
    36,
    37,
    38,
    39,
    39,
    40,
    41,
    42,
    43,
    44,
    45,
    46,
    47,
    48,
    49,
    50,
    50,
    51,
    52,
    54,
    55,
    56,
    57,
    58,
    59,
    60,
    61,
    62,
    63,
    64,
    66,
    67,
    68,
    69,
    70,
    72,
    73,
    74,
    75,
    77,
    78,
    79,
    81,
    82,
    83,
    85,
    86,
    87,
    89,
    90,
    92,
    93,
    95,
    96,
    98,
    99,
    101,
    102,
    104,
    105,
    107,
    109,
    110,
    112,
    114,
    115,
    117,
    119,
    120,
    122,
    124,
    126,
    127,
    129,
    131,
    133,
    135,
    137,
    138,
    140,
    142,
    144,
    146,
    148,
    150,
    152,
    154,
    156,
    158,
    160,
    162,
    164,
    167,
    169,
    171,
    173,
    175,
    177,
    180,
    182,
    184,
    186,
    189,
    191,
    193,
    196,
    198,
    200,
    203,
    205,
    208,
    210,
    213,
    215,
    218,
    220,
    223,
    225,
    228,
    231,
    233,
    236,
    239,
    241,
    244,
    247,
    249,
    252,
    255,
]


def gamma_correct(buffer):
    gamma_buffer = []
    for r, g, b in buffer:
        gamma_buffer.append(
            (GAMMA_CORRECTION[r], GAMMA_CORRECTION[g], GAMMA_CORRECTION[b])
        )
    return gamma_buffer


def chunked(lst, max_chunk_size=MAX_PIXELS_PER_UDP_PACKET):
    total_length = len(lst)
    num_chunks = math.ceil(total_length / max_chunk_size)
    avg_chunk_size = math.ceil(total_length / num_chunks)

    start = 0
    for _ in range(num_chunks):
        end = min(start + avg_chunk_size, total_length)
        yield start, lst[start:end]
        start = end


def _send(ip: str, port: int, packet: bytes) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        sock.sendto(packet, (ip, port))
    except socket.error as e:
        print(f"Error sending UDP packet: {e}")
    finally:
        sock.close()


def _build_packet(
    port: int, start_index: int, colors: List[Tuple[int, int, int]]
) -> bytes:
    packet = b"*P"
    packet += struct.pack("<BHH", port, start_index, len(colors))
    for r, g, b in colors:
        packet += struct.pack("<BBB", r, g, b)
    return packet


class Starpusher:
    def __init__(self, ip: str, udp_port: int, gamma_corrected=False):
        self.ip = ip
        self.udp_port = udp_port
        self.gamma_corrected = gamma_corrected

    def send(self, output_port: int, buffer: List[Tuple[int, int, int]]):
        if self.gamma_corrected:
            buffer = gamma_correct(buffer)
        for start, chunk in chunked(buffer):
            _send(self.ip, self.udp_port, _build_packet(output_port, start, chunk))
