#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Contains thread delay function
#include "driver/gpio.h" // GPIO pin controls

#define BLINK_GPIO 18 // Change this to whatever GPIO pin you're using

void app_main() {
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while(1){
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}