/*
 *  This program test several different aspects of the ESP32S2 Board

 *  By teting several different things, we are able to measure and understand 
 *  how much power the board uses. Using this, we are able to understand how good
 *  it would be in a battery operated IoT environment. 
 */

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

//Pin Toggle Stuff
#define LED 1 //Define LED Pin (Used for Board Level GPIO)

//ADC Stuff
#define DEFAULT_VREF 1100
#define NO_OF_SAMPLES 64 //Taking 64 samples per reading
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_3; //GPIO4   
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;


void app_main() {
    /////Board Level GPIO/////

    //Blink
    // To get 1kHz blink rate, need a delay of 1ms
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    while (true) {
        gpio_set_level(LED, 0);
        vTaskDelay(0.5 / portTICK_RATE_MS);
        gpio_set_level(LED, 1);
        vTaskDelay(0.5 / portTICK_RATE_MS);
    };

    //Printing over Serial(UART) over USB at 115200 (Can Be Used to Measure Sub Module Chip)
    while (true) {
        for (int i = 0; i < 10; i++) {
            printf("%d\n", i);
        }
    }

    //Continous ADC Sampling
    //Configure ADC
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    while (true) {
        uint32_t adc_reading = 0;        
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }
        adc_reading /= NO_OF_SAMPLES;
        

        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }


    /////Power Modes/////
    vTaskDelay(10000 / portTICK_RATE_MS); //10 second delay to reprogram
    const int ext_wakeup_pin = 4; //Setting Pin 4 as Wake Up Pin
    const uint64_t ext_wakeup_pin_mask = 1ULL << ext_wakeup_pin;

    printf("Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin);
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    esp_light_sleep_start(); //Light Sleep

    esp_deep_sleep_start(); // Deep Sleep
}