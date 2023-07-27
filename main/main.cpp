#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#include "sdkconfig.h"

static const char *TAG = "VLFSTAMP";

// WiFi
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#ifndef CONFIG_WIFI_MAXIMUM_RETRY
#define CONFIG_WIFI_MAXIMUM_RETRY 5
#endif

// Event handler
void event_handler(void* arg, esp_event_base_t event_base,
                   int32_t event_id, void* event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < CONFIG_WIFI_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG,"connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void wifi_init_sta()
{
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
						      ESP_EVENT_ANY_ID,
						      &event_handler,
						      NULL,
						      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
						      IP_EVENT_STA_GOT_IP,
						      &event_handler,
						      NULL,
						      &instance_got_ip));

  wifi_config_t wifi_config = { 0 };
  strlcpy((char *) wifi_config.sta.ssid, CONFIG_WIFI_SSID, sizeof(wifi_config.sta.ssid));
  strlcpy((char *) wifi_config.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_config.sta.password));
  /* Setting a password implies station will connect to all security modes including WEP/WPA.
   * However these modes are deprecated and not advisable to be used. Incase your Access point
   * doesn't support WPA2, these mode can be enabled by commenting below line */
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  wifi_config.sta.pmf_cfg.capable = true;
  wifi_config.sta.pmf_cfg.required = false;
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
  ESP_ERROR_CHECK(esp_wifi_start() );

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
   * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
					 WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
					 pdFALSE,
					 pdFALSE,
					 portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
   * happened. */
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
	     CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
	     CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }

  /* The event will not be processed after unregister */
  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
  vEventGroupDelete(s_wifi_event_group);
}

extern void setup(void);

extern void control_task(void *);
extern void udp_task(void *);
extern void i2s_task(void *);

//extern void udp_task(void *);

extern "C" void app_main(void)
{
  //esp_err_t err;

  //Initialize NVS
  nvs_flash_init();

  wifi_init_sta();

  esp_log_level_set("wifi", ESP_LOG_WARN);

  setup();

  //xTaskCreatePinnedToCore(loop_task, "loop_task", 8192, NULL, 1, NULL, 1);
  xTaskCreate(i2s_task, "i2s_task", 8192, NULL, 1, NULL);
  xTaskCreate(control_task, "control_task", 8192, NULL, 2, NULL);
  xTaskCreate(udp_task, "udp_task", 8192, NULL, 3, NULL);

  while (true) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
