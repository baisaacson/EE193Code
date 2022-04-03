#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Used for timer delay
#include "driver/i2c.h"

#define SCL_PIN 5
#define SDA_PIN 4

#define I2C_ADDR 0x91 // Bottom half of Grove water level sensor
#define READ_LEN 8 // Number of bytes to read from the sensor
#define TIMEOUT_US 10000 // 10ms

void app_main() {

    // The ESP32 has 2(?) hardware I2C modules built in
    // This number identifies which one we're using
    int i2c_hw_id = 0;

    // Configuration parameters for the I2C hardware
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Not necessary if you have external pullups
        .scl_io_num = SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,  // 400kHz, you can select another frequency if you want
    };
    i2c_param_config(i2c_hw_id, &conf);
    i2c_set_timeout(i2c_hw_id, I2C_APB_CLK_FREQ / 1000000 * TIMEOUT_US);

    i2c_driver_install(i2c_hw_id, I2C_MODE_MASTER, 0, 0, 0);

    while(1){
        // To read bytes from the sensor,
        // 1) Open the transaction
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);

        // 2) Write the 7-bit address of the device we want to talk to,
        //    plus a bit indicating R/W (1 for read, 0 for write)
        i2c_master_write_byte(cmd, I2C_ADDR << 1 | 1, true);

        // 3) Read the bytes into the buffer
        uint8_t rawdata[READ_LEN];
        i2c_master_read(cmd, rawdata, READ_LEN, I2C_MASTER_LAST_NACK);

        // 4) End the transaction
        i2c_master_stop(cmd);

        // Now that we've composed the complete I2C transaction, execute it on the hardware
        // Wait for 100ms per byte for this to complete -- this is longer than necessary
        esp_err_t err = i2c_master_cmd_begin(i2c_hw_id, cmd, 100 * (1 + READ_LEN) / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);


        // IDF 4.4 provides a simple wrapper function around all of this:
        // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html#_CPPv427i2c_master_read_from_device10i2c_port_t7uint8_tP7uint8_t6size_t10TickType_t
        // i2c_master_read_from_device(i2c_hw_id, I2C_ADDR, rawdata, READ_LEN,100 * (1 + READ_LEN) / portTICK_RATE_MS);

        for(int i = 0; i < READ_LEN; i++){
            printf("%02x ", rawdata[i]);
        }
        printf("\n");
        
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Sleep 1 second before repeating
    }

}