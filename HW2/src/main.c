#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Contains thread delay function
#include "driver/gpio.h" // GPIO pin controls


#define BLINK_GPIO 18 // Change this to whatever GPIO pin you're using

void app_main() {   
    gpio_reset_pin(BLINK_GPIO); // Reset the GPIO pin
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT); // Set GPIO pin as output

    while(1){
        (*(volatile uint32_t *)(0x3F404000 + 0x0008)) = (0x00040000); // Set GPIO18 to high
        //vTaskDelay(500 / portTICK_PERIOD_MS); 
        (*(volatile uint32_t *)(0x3F404000 + 0x000C)) = (0x00040000); // Set GPIO18 to low
        //vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}