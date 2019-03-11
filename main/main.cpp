/******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Sample program showing how to send a test message every 30 second.
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "TheThingsNetwork.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp32/ulp.h"
#include "driver/touch_pad.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"

#include "esp_system.h"
#include "rom/ets_sys.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
//extern "C" {
//#include "DHT22.h"
//}

#define SLEEP_TIME 1800

static RTC_DATA_ATTR struct timeval sleep_enter_time;

uint8_t msgData[20];

SemaphoreHandle_t xSemaphore = NULL;

void DHT_task(void *pvParameter)
{
   //float hsum=0;
   //float tsum=0;
   if( xSemaphore != NULL )
   {
       if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
       {
    	switch (esp_sleep_get_wakeup_cause()) {
    	    case ESP_SLEEP_WAKEUP_EXT1: {
    	        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
    	        if (wakeup_pin_mask != 0) {
    	            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
    	            printf("Wake up from GPIO %d\n", pin);
    	        	sprintf((char*)msgData,"pin");
    	        } else {
    	            printf("Wake up from GPIO\n");
    	        }
    	        break;
    	    }
    	    case ESP_SLEEP_WAKEUP_EXT0: {
    	        sprintf((char*)msgData,"pin");
    	        break;
    	    }
    	    case ESP_SLEEP_WAKEUP_TIMER: {
    	        sprintf((char*)msgData,"timer");
    	        break;
    	    }
    	    case ESP_SLEEP_WAKEUP_UNDEFINED:
    	    default:
    	        printf("Not a deep sleep reset\n");
    	}

	xSemaphoreGive( xSemaphore );
       }
    
   }
   vTaskDelete( NULL );
}


void sleeppa(int sec)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_EXT0: {
                printf("Wake up from GPIO EXT0 12\n");
            break;
        }
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    const int wakeup_time_sec = sec;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

//    const int ext_wakeup_pin_1 = 33;
//    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
//    const int ext_wakeup_pin_2 = 12;
//    const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;
//
//    printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
//    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
//    //esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ALL_LOW);
    
    rtc_gpio_pullup_en(GPIO_NUM_12); 
    rtc_gpio_pulldown_dis(GPIO_NUM_12); 
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_12, 0);

    
    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    //rtc_gpio_isolate(GPIO_NUM_12);

    printf("Entering deep sleep\n");
    gettimeofday(&sleep_enter_time, NULL);

    esp_deep_sleep_start();
}

#include "lora_config.h"
//// NOTE:
//// The LoRaWAN frequency and the radio chip must be configured by running 'make menuconfig'.
//// Go to Components / The Things Network, select the appropriate values and save.
//
//// Copy the below hex string from the "Device EUI" field
//// on your device's overview page in the TTN console.
//const char *devEui = "0000000000000009";
//
//// Copy the below two lines from bottom of the same page
//const char *appEui = "0000000000000005";
//const char *appKey = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
//
//// Pins and other resources
//#define TTN_SPI_HOST      HSPI_HOST
//#define TTN_SPI_DMA_CHAN  1
//#define TTN_PIN_SPI_SCLK  5
//#define TTN_PIN_SPI_MOSI  27
//#define TTN_PIN_SPI_MISO  19
//#define TTN_PIN_NSS       18
////#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
//#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
//#define TTN_PIN_RST       14
//#define TTN_PIN_DIO0      26
//#define TTN_PIN_DIO3      33

static TheThingsNetwork ttn;

const unsigned TX_INTERVAL = 60;
//static uint8_t msgData[] = "Hello, world";
//uint8_t msgData[20];
//uint8_t str[20];

void sendMessages(void* pvParameter)
{
   while (1) {
   	if( xSemaphore != NULL )
   	{
    	    if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
		printf("semaforo libero\n");
    	    	printf("Sending message...%s size:%d\n",msgData,sizeof(msgData));
    	    	TTNResponseCode res = ttn.transmitMessage(msgData, sizeof(msgData) - 1);
    	    	printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");
    	    	sleeppa(SLEEP_TIME);
    	    	//vTaskDelay(TX_INTERVAL * 1000 / portTICK_PERIOD_MS);
    	    }else{
    	    	//printf("semaforo occupato");
    	    }
    	}
    }
}

extern "C" void app_main(void)
{
    vSemaphoreCreateBinary( xSemaphore );

    nvs_flash_init();
    vTaskDelay( 1000 / portTICK_RATE_MS );
    xTaskCreate( &DHT_task, "DHT_task", 2048, NULL, 5, NULL );


    esp_err_t err;
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);

    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    // Initialize SPI bus
    spi_bus_config_t spi_bus_config;
    spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
    spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
    spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.max_transfer_sz = 0;
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);

    // Configure the SX127x pins
    ttn.configurePins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

    // The below line can be commented after the first run as the data is saved in NVS
    ttn.provision(devEui, appEui, appKey);

    printf("Joining...\n");
    if (ttn.join())
    {
        printf("Joined.\n");
        xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, NULL);
    }
    else
    {
        printf("Join failed. Goodbye\n");
	sleeppa(SLEEP_TIME);
    }
}
