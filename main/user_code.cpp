#define ESP_IDF
#define USE_SOCKETS

#include <algorithm>
#include <cfloat>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "led_strip.h"

#define PIN_NUM_FSEL GPIO_NUM_2
#define PIN_NUM_DIF  GPIO_NUM_3
#define PIN_NUM_NRST GPIO_NUM_4

#define RMT_TX_GPIO  GPIO_NUM_21
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

// Resource lock
SemaphoreHandle_t xSem = NULL;

// LED
led_strip_handle_t led_strip;

// I2S
i2s_chan_handle_t rx_handle;

#if CONFIG_I2S_ONLY_RIGHT
#define i2s_slot_mode I2S_SLOT_MODE_MONO
#define i2s_slot_mask I2S_STD_SLOT_RIGHT
#elif CONFIG_I2S_ONLY_LEFT
#define i2s_slot_mode I2S_SLOT_MODE_MONO
#define i2s_slot_mask I2S_STD_SLOT_LEFT
#else //CONFIG_I2S_STEREO
#define i2s_slot_mode I2S_SLOT_MODE_STEREO
#define i2s_slot_mask I2S_STD_SLOT_BOTH
#endif


// UDP client
int sock = -1;
in_addr_t udp_addr;
bool request_udp_reconnect = false;

void udp_init(void)
{
  struct sockaddr_in caddr;
  struct sockaddr_in saddr;
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    printf("UDP socket can't be opened.\n");
    return;
  }

  memset((char *) &caddr, 0, sizeof(caddr));
  caddr.sin_family = AF_INET;
  caddr.sin_addr.s_addr = htonl(INADDR_ANY);
  caddr.sin_port = htons(CONFIG_UDP_PORT);

  memset((char *) &saddr, 0, sizeof(saddr));
  saddr.sin_family = AF_INET;
  xSemaphoreTake(xSem, portMAX_DELAY);
  saddr.sin_addr.s_addr = udp_addr;
  xSemaphoreGive(xSem);
  saddr.sin_port = htons(CONFIG_UDP_PORT);

  if (bind (sock, (struct sockaddr *)&caddr, sizeof(caddr)) < 0) {
    printf("Failed to bind UDP socket.\n");
    close(sock);
    sock = -1;
    return;
  }

  if (connect(sock, (struct sockaddr *) &saddr, sizeof(saddr)) < 0) {
    printf("Failed to connect socket.\n");
    close(sock);
    sock = -1;
    return;
  }
}

void udp_reconnect(void)
{
  led_strip_set_pixel(led_strip, 0, 32, 0, 0);
  led_strip_refresh(led_strip);
  if (sock >= 0)
    close(sock);
  sock = -1;
  int retry = 5;
  while (sock < 0 && retry-- > 0) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    udp_init();
    if (sock >= 0) {
      led_strip_set_pixel(led_strip, 0, 0, 32, 32);
      led_strip_refresh(led_strip);
      return;
    }
  }
  // Last resort
  esp_restart();
}
  
#define SAMPLE_SIZE 1024
uint8_t i2s_readraw_buff[SAMPLE_SIZE*2*4];
size_t bytes_read;

QueueHandle_t pkt_queue;
#define PKT_QSIZE 2
#define PACKET_SIZE sizeof(i2s_readraw_buff)

bool enable_fft = true;

extern "C" {
  void rdft(int n, int isgn, float *a, int *ip, float *w);
}

int ip[32];
float w[SAMPLE_SIZE/2];
float fftio[SAMPLE_SIZE];
float bh4[SAMPLE_SIZE] = {
#include "bh.inc"
};

void setup()
{
  // Mutex
  xSem = xSemaphoreCreateMutex();

  // Queue
  pkt_queue = xQueueCreate(PKT_QSIZE, PACKET_SIZE);

  // GPIO
  gpio_set_direction(PIN_NUM_FSEL, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_NUM_DIF, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_NUM_NRST, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_NUM_FSEL, 0);
  gpio_set_level(PIN_NUM_DIF, 0);
  gpio_set_level(PIN_NUM_NRST, 0);
  gpio_set_level(PIN_NUM_NRST, 1);
  
  // LED
  led_strip_config_t strip_config = {
    .strip_gpio_num = RMT_TX_GPIO,
    .max_leds = 1,
    .led_pixel_format = LED_PIXEL_FORMAT_GRB,
    .led_model = LED_MODEL_WS2812,
  };
  // LED strip backend configuration: RMT
  led_strip_rmt_config_t rmt_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = LED_STRIP_RMT_RES_HZ,
  };

  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  led_strip_clear(led_strip);
  vTaskDelay(pdMS_TO_TICKS(500));
  led_strip_set_pixel(led_strip, 0, 0, 0, 32);
  led_strip_refresh(led_strip);

  // FFT
  ip[0] = 0;
  rdft(SAMPLE_SIZE, 1, fftio, ip, w);

  // UDP
  udp_addr = inet_addr(CONFIG_UDP_ADDRESS);
  udp_reconnect();

  // I2S
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, NULL, &rx_handle);
  i2s_std_config_t std_rx_cfg = {
    .clk_cfg = {
      .sample_rate_hz = CONFIG_I2S_SAMPLE_RATE,
      .clk_src = I2S_CLK_SRC_PLL_160M,
      .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    },
    .slot_cfg = {
      .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
      .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
      .slot_mode = i2s_slot_mode,
      .slot_mask = i2s_slot_mask,
      .ws_width = 32,
      .ws_pol = false,
      .bit_shift = false,
    },
    .gpio_cfg = {
      .mclk = (gpio_num_t)CONFIG_I2S_MCLK_GPIO,
      .bclk = (gpio_num_t)CONFIG_I2S_BCK_GPIO,
      .ws = (gpio_num_t)CONFIG_I2S_WS_GPIO,
      .dout = I2S_GPIO_UNUSED,
      .din = (gpio_num_t)CONFIG_I2S_DATAIN_GPIO,
      .invert_flags = {
	.mclk_inv = false,
	.bclk_inv = false,
	.ws_inv = false,
      },
    },
  };
  /* Initialize the channel */
  i2s_channel_init_std_mode(rx_handle, &std_rx_cfg);
  i2s_channel_enable(rx_handle);

}

int count;

int32_t get_s32(uint8_t *p)
{
  return (int32_t)(p[0] + (p[1] << 8) + (p[2] << 16) + (p[3] << 24));
}

void loop()
{
  
  i2s_channel_read(rx_handle, i2s_readraw_buff, sizeof(i2s_readraw_buff),
		   &bytes_read, 1000);

  xQueueSend(pkt_queue, (void *)i2s_readraw_buff, (TickType_t)0);

  count++;
}

// Setup TCP server
static const char *TAG = "VLFSTAMP";

void control_task(void *arg)
{
  struct sockaddr_in saddr;
  struct sockaddr_in caddr;
  saddr.sin_family = AF_INET;
  saddr.sin_addr.s_addr = htonl(INADDR_ANY);
  saddr.sin_port = htons(CONFIG_TCP_PORT);

  int so = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (so < 0) {
    ESP_LOGE(TAG, "TCP socket can't be opened");
    vTaskDelete(NULL);
  }
  
  int rc = bind(so, (struct sockaddr *)&saddr, sizeof(saddr));
  if (rc < 0) {
    ESP_LOGE(TAG, "TCP socket can't be bound");
    vTaskDelete(NULL);
  }

  rc = listen(so, 5);
  if (rc < 0) {
    ESP_LOGE(TAG, "TCP socket can't be listened");
    vTaskDelete(NULL);
  }

  while (true) {
    socklen_t caddrlen = sizeof(caddr);
    int csock = accept(so, (struct sockaddr *)&caddr, &caddrlen);
    if (csock < 0) {
      ESP_LOGE(TAG, "Failed to accept (%s)", strerror(errno));
      vTaskDelete(NULL);
    }

    printf("Request\n");

    uint8_t buf[512];
    ssize_t n = 0;

    while (true) {
      n = recv(csock, buf, sizeof(buf), 0);
      if (n < 0) {
	ESP_LOGE(TAG, "Failed to recieve (%s)", strerror(errno));
	break;
      } else if (n == 0) {
	break;
      }

      uint8_t *e;
      if ((e = (uint8_t *)memchr(buf, '\n', n)) == NULL) {
	//printf("Bad command line\n");
	continue;
      }
      *e = '\0';

      const char *cmd = (const char *)buf;
      if (strncmp(cmd, "mode raw", 8) == 0) {
	xSemaphoreTake(xSem, portMAX_DELAY);
	enable_fft = false;
	xSemaphoreGive(xSem);
	send(csock, "ok\n", 3, 0);
      } else if (strncmp(cmd, "mode fft", 8) == 0) {
	xSemaphoreTake(xSem, portMAX_DELAY);
	enable_fft = true;
	xSemaphoreGive(xSem);
	send(csock, "ok\n", 3, 0);
      } else if (strncmp(cmd, "mode", 4) == 0) {
	send(csock, (enable_fft ? "fft\n" : "raw\n"), 4, 0);
      } else if (strncmp(cmd, "addr ", 5) == 0) {
	in_addr_t newaddr = inet_addr(cmd+5);
	if (INADDR_NONE != newaddr) {
	  xSemaphoreTake(xSem, portMAX_DELAY);
	  udp_addr = newaddr;
	  request_udp_reconnect = true;
	  xSemaphoreGive(xSem);
	  send(csock, "ok\n", 3, 0);
	} else {
	  send(csock, "bad addr\n", 9, 0);
	}
      } else if (strncmp(cmd, "help", 4) == 0) {
	const char *msg = "Command list:\n mode [fft|raw]\n addr aa.bb.cc.dd\n help\n";
	send(csock, msg, strlen(msg), 0);
      } else {
	send(csock, "??? ", 4, 0);
	send(csock, cmd, strlen(cmd), 0);
	send(csock, "\n", 1, 0);
	//printf("%s unimplemented\n", cmd);
      }
    }

    //printf("End of request\n");
    close(csock);
  }
    
}

uint8_t raw_buff[PACKET_SIZE];

void udp_task(void *)
{
  while (1) {
    bool do_fft = true;
    bool do_udp_reconnect = false;
    xSemaphoreTake(xSem, portMAX_DELAY);
    do_fft = enable_fft;
    if (request_udp_reconnect) {
      do_udp_reconnect = true;
      request_udp_reconnect = false;
    }
    xSemaphoreGive(xSem);

    if (do_udp_reconnect) {
      do_udp_reconnect = false;
      udp_reconnect();
    }

    // wait queue
    xQueueReceive(pkt_queue, raw_buff, portMAX_DELAY);

    if (sock >= 0 && do_fft) {
      // right_ch .* bh4
      for (int i = 0; i < SAMPLE_SIZE; i++)
	fftio[i] = get_s32(raw_buff + 8*i + 4) * bh4[i];
      rdft(SAMPLE_SIZE, 1, fftio, ip, w);

      int n = send(sock, (uint8_t *)fftio, sizeof(fftio), 0);
      if (n < 0) {
	printf("send error: %s\n", strerror(errno));
	if (errno == ECONNRESET || errno == ENOTCONN) {
	  udp_reconnect();
	}
      }
    } else if (sock >= 0) {
      int n = send(sock, raw_buff, sizeof(i2s_readraw_buff), 0);
      if (n < 0) {
	printf("send error: %s\n", strerror(errno));
	if (errno == ECONNRESET || errno == ENOTCONN) {
	  udp_reconnect();
	}
      }
    } else {
      ;
    }
  }
}
