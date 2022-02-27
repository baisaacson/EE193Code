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

#define BILLION  1000000000.0


//ADC Channels
#define ADC2_EXAMPLE_CHAN0          ADC1_CHANNEL_0

static const adc_bits_width_t width = ADC_WIDTH_BIT_13;


static const char *TAG_CH[2][10] = {{"ADC1_CH2"}, {"ADC2_CH0"}};

static int adc_raw[2][10];

void app_main(void)
{
   // clock_t startClock, endClock;
    //double time;

    struct timespec start, end;

    while (1) {
        //startClock = clock();
            clock_gettime(CLOCK_REALTIME, &start);

        adc_raw[0][0] = adc2_get_raw(ADC2_EXAMPLE_CHAN0, width,&adc_raw[1][0]);
            clock_gettime(CLOCK_REALTIME, &end);
double time_spent = (end.tv_sec - start.tv_sec) +
                        (end.tv_nsec - start.tv_nsec) / BILLION;
      //  endClock = clock();
      //  time = (double)(startClock - endClock);
        ESP_LOGI(TAG_CH[0][0], "raw  data: %d", adc_raw[1][0]);
                ESP_LOGI(TAG_CH[0][0], "The elapsed time is %f seconds", time_spent);

       // ESP_LOGI(TAG_CH[0][0], "time: %f", time);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//Have removed calibration of the data, checks, initializations, etc.
//Added width for resolution. Only avalible for ADC2
//added clock_gettime() for time stamping. Tried clock() but it was not accurate enough.