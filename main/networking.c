#include <esp_eth.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <lwip/err.h>
#include <lwip/ip4_addr.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>

#include "buffered_led_strips.h"
#include "networking.h"
#include "peripherals.h"
#include "udp_protocol.h"

static const char *TAG = "networking";

#define WT32_ETH01_PHY_ADDR 1
#define WT32_ETH01_RESET_GPIO_NUM 16

#define MAX_UDP_PACKET_SIZE 1472
#define LISTEN_UDP_PORT 6868

#define DISCOVERY_MULTICAST_GROUP "239.10.11.12"
#define DISCOVERY_MULTICAST_PORT 21059
#define DISCOVERY_MULTICAST_TTL 5
#define DISCOVERY_BROADCAST_INTERVAL_MILLIS 1000

#define WATCHDOG_INTERVAL_MILLIS 5000

TimerHandle_t udp_watchdog_timer;
TimerHandle_t udp_discovery_timer;

uint8_t device_id;

void create_static_ip_from_device_id(uint8_t device_id,
                                     char *buffer,
                                     size_t size) {
  snprintf(buffer, size, "10.1.1.%d", device_id + 100);
}

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg,
                              esp_event_base_t event_base,
                              int32_t event_id,
                              void *event_data) {
  uint8_t mac_addr[6] = {0};
  /* we can get the ethernet driver handle from event data */
  esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

  switch (event_id) {
  case ETHERNET_EVENT_CONNECTED:
    esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
    ESP_LOGI(TAG, "Ethernet Link: UP");
    ESP_LOGI(TAG,
             "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0],
             mac_addr[1],
             mac_addr[2],
             mac_addr[3],
             mac_addr[4],
             mac_addr[5]);
    break;
  case ETHERNET_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "Ethernet Link: DOWN");
    break;
  case ETHERNET_EVENT_START:
    ESP_LOGI(TAG, "Ethernet: STARTED");
    break;
  case ETHERNET_EVENT_STOP:
    ESP_LOGI(TAG, "Ethernet: STOPPED");
    break;
  default:
    break;
  }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg,
                                 esp_event_base_t event_base,
                                 int32_t event_id,
                                 void *event_data) {
  ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
  const esp_netif_ip_info_t *ip_info = &event->ip_info;

  ESP_LOGI(TAG, "Ethernet Got IP Address");
  ESP_LOGI(TAG, "~~~~~~~~~~~");
  ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
  ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
  ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
  ESP_LOGI(TAG, "~~~~~~~~~~~");
}

static void set_static_ip(esp_netif_t *netif, char *ip_address) {
  if (esp_netif_dhcpc_stop(netif) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to stop dhcp client");
    return;
  }
  esp_netif_ip_info_t ip;
  memset(&ip, 0, sizeof(esp_netif_ip_info_t));
  ip.ip.addr = ipaddr_addr(ip_address);
  ip.netmask.addr = ipaddr_addr("255.255.255.0");
  ip.gw.addr = ipaddr_addr("10.1.1.1");
  if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set ip info");
    return;
  }
}

static void udp_watchdog_callback(TimerHandle_t xTimer) {
  ESP_LOGE(TAG, "UDP watchdog tripped");
  indicator_led_set(0);
  // If we haven't received UDP packets in a while, then turn off
  // all of the LEDs.
  if (buffered_led_strips_acquire_buffers_mutex()) {
    if (device_id == TEST_PATTERN_DEVICE_ID) {
      // Device ID 15 UDP watchdog sets all colors to white.
      buffered_led_strips_reset(255, 255, 255, 255);
    } else {
      buffered_led_strips_reset(0, 0, 0, 0);
    }
    buffered_led_strips_release_buffers_mutex();
  }
}

static void udp_discovery_callback(TimerHandle_t xTimer) {
  struct sockaddr_in saddr = {0};
  int sock = -1;
  int err = 0;

  char message[7] = {'S', 'P', device_id, 10, 1, 1, 100 + device_id};

  sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock < 0) {
    ESP_LOGE(TAG, "Failed to create socket. Error %d", errno);
    return;
  }

  saddr.sin_family = AF_INET;
  saddr.sin_port = htons(DISCOVERY_MULTICAST_PORT);
  saddr.sin_addr.s_addr = inet_addr(DISCOVERY_MULTICAST_GROUP);

  err = sendto(sock,
               message,
               sizeof(message),
               0,
               (struct sockaddr *)&saddr,
               sizeof(saddr));
  if (err < 0) {
    ESP_LOGE(TAG, "Failed to send to socket. Error %d", errno);
    return;
  }
  close(sock);
}

static void udp_server_task(void *pvParameters) {
  uint8_t rx_buffer[MAX_UDP_PACKET_SIZE];
  struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
  socklen_t socklen = sizeof(source_addr);

  struct sockaddr_in dest_addr;
  dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(LISTEN_UDP_PORT);

  while (1) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      break;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
      ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket bound, port %d", LISTEN_UDP_PORT);
    while (1) {
      ssize_t size = recvfrom(sock,
                              rx_buffer,
                              sizeof(rx_buffer),
                              0,
                              (struct sockaddr *)&source_addr,
                              &socklen);
      if (size < 0) {
        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        break;
      } else {
        if (!xTimerIsTimerActive(udp_watchdog_timer)) {
          ESP_LOGI(TAG, "UDP watchdog reset");
        }
        xTimerReset(udp_watchdog_timer, 0);

        indicator_led_set(1);
        process_udp_packet(rx_buffer, size);
      }
    }
    if (sock != -1) {
      ESP_LOGE(TAG, "Shutting down socket and restarting...");
      shutdown(sock, 0);
      close(sock);
    }
  }
  vTaskDelete(NULL);
}

void networking_initialize(uint8_t init_device_id) {
  device_id = init_device_id;

  char static_ip_address[32];
  create_static_ip_from_device_id(
      device_id, static_ip_address, sizeof(static_ip_address));

  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
  esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);

  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  // The WT32 ETH0 doesn't use 100% default settings. We
  // have to hard code the PHY device address and bind
  // the PHY reset to GPIO pin 0.
  phy_config.phy_addr = WT32_ETH01_PHY_ADDR;
  phy_config.reset_gpio_num = WT32_ETH01_RESET_GPIO_NUM;
  esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);

  esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
  esp_eth_handle_t eth_handle = NULL;
  esp_eth_driver_install(&config, &eth_handle);

  esp_event_loop_create_default();

  esp_event_handler_register(
      ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);

  esp_netif_init();

  esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();

  esp_netif_t *eth_netif = esp_netif_new(&cfg);
  set_static_ip(eth_netif, static_ip_address);

  esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle));
  esp_event_handler_register(
      IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL);
  esp_eth_start(eth_handle);

  udp_watchdog_timer = xTimerCreate("udp_watchdog",
                                    pdMS_TO_TICKS(WATCHDOG_INTERVAL_MILLIS),
                                    pdFALSE,
                                    NULL,
                                    udp_watchdog_callback);
  xTimerStart(udp_watchdog_timer, 0);

  udp_discovery_timer =
      xTimerCreate("udp_discovery",
                   pdMS_TO_TICKS(DISCOVERY_BROADCAST_INTERVAL_MILLIS),
                   pdTRUE,
                   NULL,
                   udp_discovery_callback);
  xTimerStart(udp_discovery_timer, 0);

  xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
}