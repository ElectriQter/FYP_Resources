/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_board_init.h"
#include "speech_commands_action.h"
#include "model_path.h"
#include "esp_process_sdkconfig.h"

//To control the GPIO Pins on the ESP32S3
#include "driver/gpio.h"

int detect_flag = 0;
static esp_afe_sr_iface_t *afe_handle = NULL;
static volatile int task_flag = 0;
srmodel_list_t *models = NULL;
static int play_voice = -2;

void play_music(void *arg)
{
    while (task_flag) {
        switch (play_voice) {
        case -2:
            vTaskDelay(10);
            break;
        case -1:
            wake_up_action();
            play_voice = -2;
            break;
        default:
            speech_commands_action(play_voice);
            play_voice = -2;
            break;
        }
    }
    vTaskDelete(NULL);
}

void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_channel_num(afe_data);
    int feed_channel = esp_get_feed_channel();
    assert(nch <= feed_channel);
    int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    assert(i2s_buff);

    while (task_flag) {
        esp_get_feed_data(false, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);

        afe_handle->feed(afe_data, i2s_buff);
    }
    if (i2s_buff) {
        free(i2s_buff);
        i2s_buff = NULL;
    }
    vTaskDelete(NULL);
}

void detect_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    printf("multinet:%s\n", mn_name);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *model_data = multinet->create(mn_name, 6000);
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    esp_mn_commands_update_from_sdkconfig(multinet, model_data); // Add speech commands from sdkconfig
    assert(mu_chunksize == afe_chunksize);
    //print active speech commands
    multinet->print_active_speech_commands(model_data);
    
    printf("------------detect start------------\n");
    
    rst();
    ledStandby();
    while (task_flag) {
        afe_fetch_result_t* res = afe_handle->fetch(afe_data); 
        if (!res || res->ret_value == ESP_FAIL) {
            printf("fetch error!\n");
            break;
        }

        if (res->wakeup_state == WAKENET_DETECTED) {
            printf("WAKEWORD DETECTED\n");
            ledDetected();
	    multinet->clean(model_data);
        } else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
            play_voice = -1;
            detect_flag = 1;
            printf("AFE_FETCH_CHANNEL_VERIFIED, channel index: %d\n", res->trigger_channel_id);
            // afe_handle->disable_wakenet(afe_data);
            // afe_handle->disable_aec(afe_data);
        }

        if (detect_flag == 1) {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

            if (mn_state == ESP_MN_STATE_DETECTING) {
                continue;
            }

            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                for (int i = 0; i < mn_result->num; i++) {
                    printf("TOP %d, command_id: %d, phrase_id: %d, string: %s, prob: %f\n", 
                    i+1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->string, mn_result->prob[i]);
                }
                if (mn_result->num>0)
                {
                    switch (mn_result->command_id[0]) 
                    {
                        case 1:
                            //detect_flag = 2;
                            ledHW();
                            printf("Hello World\n");
                            break;
                        case 2:
                            //detect_flag = 2;
                            ledOn();
                            printf("LED ON\n");
                            break;
                        case 3:
                            //detect_flag = 2;
                            ledB();
                            printf("LED Blue\n");
                            break;
                        case 4:
                            //detect_flag = 2;
                            ledR();
                            printf("LED Red\n");

                            break;
                        case 5:
                            //detect_flag = 2;
                            ledG();
                            printf("LED Green\n");
                            break;
                        case 6:
                            //detect_flag = 2;
                            ledOff();
                            printf("LED OFF\n");
                            break;
                        case 7:
                            speakThrow();   //RED LED
                            printf("Throw the ball\n");
                            break;
                        case 8: 
                            speakRoll();    // BLUE LED
                            printf("Roll the ball\n");
                            break;
                        case 9:
                            speakNot();     // GREEN LED
                            printf("No action\n");
                            break;
                        case 10:
                            speakThrown();   // RED & GREEN LED
                            printf("The ball was thrown\n");
                            break;
                        case 11:
                            speakRolled();   // BLUE AND GREEN LED
                            printf("The ball was rolled\n");
                            break;
                        case 12:
                            speakNoted();    // RBG ON
                            printf("No action on ball\n");
                            break;

                        default:
                            detect_flag = 0;  // Reset flag to wait for next detection
                            printf("Unrecognized command\n");
                            break;

                    }
                }
                printf("-----------listening-----------\n");
            }

            if (mn_state == ESP_MN_STATE_TIMEOUT) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                printf("timeout, string:%s\n", mn_result->string);
                afe_handle->enable_wakenet(afe_data);
                detect_flag = 0;
                
                printf("\n-----------awaits to be waken up-----------\n");
                ledStandby();
                continue;
            }
        }
    }
    if (model_data) {
        multinet->destroy(model_data);
        model_data = NULL;
    }
    printf("detect exit\n");
    vTaskDelete(NULL);
}

void rst()
{
    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_18, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_OUTPUT);      //red for testing
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);      //blue
    gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT);      //green

    gpio_set_level(GPIO_NUM_16, 1);
    gpio_set_level(GPIO_NUM_17, 1);
    gpio_set_level(GPIO_NUM_18, 1);
    gpio_set_level(GPIO_NUM_1, 0);
    gpio_set_level(GPIO_NUM_2, 0);
    gpio_set_level(GPIO_NUM_21, 0);

}


// define the Pin 20 to be flag for output 
void speaksetup()
{
    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_OUTPUT);      //red for testing
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);      //blue
    gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT);      //green
}

void speakThrow()      // ball thrown
{
    speaksetup();
    gpio_set_level(GPIO_NUM_1, 1);
    gpio_set_level(GPIO_NUM_2, 0);
    gpio_set_level(GPIO_NUM_21, 0);
    pinrst();

}

void speakRoll()       // ball rolled
{ 
    speaksetup();
    gpio_set_level(GPIO_NUM_1, 0);
    gpio_set_level(GPIO_NUM_2, 1);
    gpio_set_level(GPIO_NUM_21, 0);
    pinrst();
}

void speakNot()     //no action
{ 
    speaksetup();
    gpio_set_level(GPIO_NUM_1, 0);
    gpio_set_level(GPIO_NUM_2, 0);
    gpio_set_level(GPIO_NUM_21, 1);
    pinrst();
}

void speakThrown()
{
    speaksetup();
    gpio_set_level(GPIO_NUM_1, 1);
    gpio_set_level(GPIO_NUM_2, 0);
    gpio_set_level(GPIO_NUM_21, 1);
    pinrst();
}

void speakRolled()
{ 
    speaksetup();
    gpio_set_level(GPIO_NUM_1, 0);
    gpio_set_level(GPIO_NUM_2, 1);
    gpio_set_level(GPIO_NUM_21, 1);
    pinrst();
}

void speakNoted()
{ 
    speaksetup();
    gpio_set_level(GPIO_NUM_1, 1);
    gpio_set_level(GPIO_NUM_2, 1);
    gpio_set_level(GPIO_NUM_21, 1);
    pinrst();
}

void LEDsetup()
{
    gpio_set_direction(GPIO_NUM_18, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
    //Blue = 16, Green = 17, Red = 18
}
void pinrst()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(GPIO_NUM_1, 0);
    gpio_set_level(GPIO_NUM_2, 0);
    gpio_set_level(GPIO_NUM_21, 0);
}
void LLEDrst()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(GPIO_NUM_16, 1);
    gpio_set_level(GPIO_NUM_17, 1);
    gpio_set_level(GPIO_NUM_18, 1);
}
void ledHW()
{
    LEDsetup();
    gpio_set_level(GPIO_NUM_16, 0);
    gpio_set_level(GPIO_NUM_17, 1);
    gpio_set_level(GPIO_NUM_18, 0);
    LLEDrst();
}

void ledOn()
{
    LEDsetup();
    gpio_set_level(GPIO_NUM_16, 0);
    gpio_set_level(GPIO_NUM_17, 0);
    gpio_set_level(GPIO_NUM_18, 0);
    LLEDrst();
}

void ledOff()
{
    LEDsetup();
    gpio_set_level(GPIO_NUM_16, 1);
    gpio_set_level(GPIO_NUM_17, 1);
    gpio_set_level(GPIO_NUM_18, 1);
}

void ledB()
{
    LEDsetup();
    gpio_set_level(GPIO_NUM_16, 0);
    gpio_set_level(GPIO_NUM_17, 1);
    gpio_set_level(GPIO_NUM_18, 1);
    LLEDrst();
}

void ledR()
{
    LEDsetup();
    gpio_set_level(GPIO_NUM_16, 1);
    gpio_set_level(GPIO_NUM_17, 1);
    gpio_set_level(GPIO_NUM_18, 0);
    LLEDrst();
}

void ledG()
{
    LEDsetup();
    gpio_set_level(GPIO_NUM_16, 1);
    gpio_set_level(GPIO_NUM_17, 0);
    gpio_set_level(GPIO_NUM_18, 1);
    LLEDrst();
}

void ledStandby()
{
    LEDsetup();
    gpio_set_level(GPIO_NUM_16, 1);
    gpio_set_level(GPIO_NUM_17, 1);
    gpio_set_level(GPIO_NUM_18, 0);
    //LLEDrst();
}

void ledDetected()
{
    LEDsetup();
    gpio_set_level(GPIO_NUM_16, 1);
    gpio_set_level(GPIO_NUM_17, 0);
    gpio_set_level(GPIO_NUM_18, 1);
    //LLEDrst();
}


void app_main()
{
    /*
    //ESP NOW 
    wifi_sta_init();

    esp_now_init();
    esp_now_register_send_cb(esp_now_send_callback);
    esp_now_peer_info_t peer_info = {0};
    peer_info.channel = 1; 
    peer_info.encrypt = false;
    uint8_t esp_mac[6] = {0x8c,0xbf,0xea,0x03,0xb6,0x60};

    memcpy(peer_info.peer_addr, esp_mac, 6);
    esp_now_add_peer(&peer_info);
    while(1)
    {
        esp_err_t err = esp_now_send(esp_mac, (uint8_t *) "Sending via ESP-NOW", strlen("Sending via ESP-NOW"));
            ESP_LOGI(TAG,"esp now status : %s", esp_err_to_name(err));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    //End of ESP now
    */
    models = esp_srmodel_init("model"); // partition label defined in partitions.csv
    ESP_ERROR_CHECK(esp_board_init(AUDIO_HAL_16K_SAMPLES, 1, 16));
    // ESP_ERROR_CHECK(esp_sdcard_init("/sdcard", 10));
#if defined CONFIG_ESP32_KORVO_V1_1_BOARD
    led_init();
#endif

#if CONFIG_IDF_TARGET_ESP32
    printf("This demo only support ESP32S3\n");
    return;
#else 
    afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
#endif

    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    afe_config.wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);;
#if defined CONFIG_ESP32_S3_BOX_BOARD || defined CONFIG_ESP32_S3_EYE_BOARD || CONFIG_ESP32_S3_DEVKIT_C
    afe_config.aec_init = false;
    #if defined CONFIG_ESP32_S3_EYE_BOARD || CONFIG_ESP32_S3_DEVKIT_C
        afe_config.pcm_config.total_ch_num = 2;
        afe_config.pcm_config.mic_num = 1;
        afe_config.pcm_config.ref_num = 1;
    #endif
#endif
    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);

    task_flag = 1;
    xTaskCreatePinnedToCore(&detect_Task, "detect", 8 * 1024, (void*)afe_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(&feed_Task, "feed", 8 * 1024, (void*)afe_data, 5, NULL, 0);
#if defined  CONFIG_ESP32_S3_KORVO_1_V4_0_BOARD
    xTaskCreatePinnedToCore(&led_Task, "led", 2 * 1024, NULL, 5, NULL, 0);
#endif
#if defined  CONFIG_ESP32_S3_KORVO_1_V4_0_BOARD || CONFIG_ESP32_S3_KORVO_2_V3_0_BOARD || CONFIG_ESP32_KORVO_V1_1_BOARD  || CONFIG_ESP32_S3_BOX_BOARD
    xTaskCreatePinnedToCore(&play_music, "play", 4 * 1024, NULL, 5, NULL, 1);
#endif

    // // You can call afe_handle->destroy to destroy AFE.
    // task_flag = 0;

    // printf("destroy\n");
    // afe_handle->destroy(afe_data);
    // afe_data = NULL;
    // printf("successful\n");
}
