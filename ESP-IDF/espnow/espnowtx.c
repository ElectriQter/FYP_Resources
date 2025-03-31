// Transmission Text

#include <stdio.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "string.h"
#include "nvs_flash.h"
#include "esp_now.h"

static const char* TAG = "ESP-NOW TX";              //declare identifier

// Define a structure that holds the data to send
typedef struct struct_message {
    char a[100];    // String data
    int b;         // Integer
    float c;       // Float
    bool d;        // Boolean
} struct_message;

struct_message msg;
//Function to send 
void esp_now_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    ESP_LOGI(TAG,"tx cb");
}

//Set up wifi and MAC address
void wifi_sta_init(void)
{
    esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    nvs_flash_erase();
    ret = nvs_flash_init();
  }
  esp_netif_init();
  esp_event_loop_create_default();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_start();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  uint8_t my_esp_mac[6] = {0xA0,0xA3,0xB3,0xAB,0x19,0x44};
  esp_read_mac(my_esp_mac, ESP_MAC_WIFI_STA);
  ESP_LOGI(TAG, "peer mac " MACSTR "", my_esp_mac[0], my_esp_mac[1], my_esp_mac[2], my_esp_mac[3], my_esp_mac[4], my_esp_mac[5]);
}
void app_main(void)
{
    struct_message msg;
    //char input_message[100];
    snprintf(msg.a, sizeof(msg.a), "Hello, this is a dynamic message!"); // Example message
    msg.b = 42;
    msg.c = 3.14;
    msg.d = true;

    wifi_sta_init();

    esp_now_init();
    esp_now_register_send_cb(esp_now_send_callback);
    esp_now_peer_info_t peer_info = {0};
    peer_info.channel = 1; 
    peer_info.encrypt = false;
    uint8_t esp_mac[6] = {0xA0,0xA3,0xB3,0xAB,0x19,0x44};

    memcpy(peer_info.peer_addr, esp_mac, 6);
    esp_now_add_peer(&peer_info);
    while(1)
    {
        esp_err_t err = esp_now_send(esp_mac, (uint8_t *)&msg, sizeof(msg));
            ESP_LOGI(TAG,"esp now status : %s", esp_err_to_name(err));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}