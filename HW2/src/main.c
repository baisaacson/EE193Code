/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

//Need to use 115200 Baud rate

#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "esp_system.h"
#include <time.h>

#define BILLION  1000000000.0 // 1 billion


//ADC Channels
#define ADC1_EXAMPLE_CHAN0          ADC1_CHANNEL_2

//ADC Sampling
static const adc_bits_width_t width = ADC_WIDTH_BIT_DEFAULT;

static const char *TAG_CH[2][10] = {{"ADC1_CH2"}, {"ADC2_CH0"}}; //ADC Channels
static int adc_raw[2][10]; //ADC raw data

void app_main(void)
{
    struct timespec start, end; //Time variables
    adc1_config_width(width); //Configure ADC1 to read 12 bits
    while (1) {
        clock_gettime(CLOCK_REALTIME, &start); //Start time
        adc_raw[0][0] = adc1_get_raw(ADC1_EXAMPLE_CHAN0); //Read ADC1
        clock_gettime(CLOCK_REALTIME, &end); //End time
        double time_spent = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / BILLION; //Calculate time spent
 
        ESP_LOGI(TAG_CH[0][0], "raw  data: %d", adc_raw[0][0]); //Print raw data
        ESP_LOGI(TAG_CH[0][0], "The elapsed time is %f seconds", time_spent); //Print time spent

        vTaskDelay(pdMS_TO_TICKS(1000)); //Delay 1 second
    }
}

//Have removed calibration of the data, checks, initializations, etc.
//Added width for resolution. Only avalible for ADC2
//added clock_gettime() for time stamping. Tried clock() but it was not accurate enough.
//Changing the bit width other than default using adc1_config_width() is not supported.
