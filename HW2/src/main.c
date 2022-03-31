/* i2c - Simple example
   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.
   The sensor used in this example is a MPU9250 inertial measurement unit.
   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples
   See README.md file to get detailed usage of this example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
// #include <stdio.h>
// #include "esp_log.h"
// #include "driver/i2c.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "driver/i2c.h"

// static const char *TAG = "i2c-simple-example";

// #define I2C_MASTER_SCL_IO           33      /*!< GPIO number used for I2C master clock */
// #define I2C_MASTER_SDA_IO           34      /*!< GPIO number used for I2C master data  */
// #define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
// #define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
// #define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_TIMEOUT_MS       1000

// #define SENSOR_ADDR                 0x91        /*!< Slave address of the MPU9250 sensor */
// #define TEMP_REG                    0x00 //Temperature Register From Sensor

// #define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

// #define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
// #define MPU9250_RESET_BIT                   7            /*!< Bit mask of the reset bit */


// #define BH1750_SENSOR_ADDR CONFIG_BH1750_ADDR   /*!< slave address for BH1750 sensor */
// #define BH1750_CMD_START CONFIG_BH1750_OPMODE   /*!< Operation mode */
// #define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
// #define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
// #define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
// #define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
// #define ACK_VAL 0x0                             /*!< I2C ack value */
// #define NACK_VAL 0x1                            /*!< I2C nack value */

// #define DATA_LENGTH 16                          /*!< I2C data length */

//I2C SENSOR STUFF//
#define I2C_MASTER_SCL_IO           5      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           4      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
#define SENSOR_ADDR                0x91        /*!< Slave address of the temp sensor */
#define TEMP_REG                    0x00 //Temperature Register From Sensor


uint8_t myPow (uint8_t base, uint8_t exponent) {

    uint8_t result = 1;

    for (uint8_t i = 0; i < exponent; i++) {
        result *= base;
    }

    return result;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Read a sequence of bytes from a temp sensor registers
 */
static esp_err_t temp_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{   
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    //i2c_master_write_byte(cmd,(SENSOR_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    //i2c_master_write_byte(cmd, 0x90, ACK_CHECK_EN); //0x91 is the slave address hardcoded (the 1 in the LSB tells that it's a read)
    //i2c_master_write_byte(cmd, TEMP_REG, ACK_CHECK_EN); //0x00 is the slave address hardcoded (the 1 in the LSB tells that it's a read)
    i2c_master_write_byte(cmd, 0x91, ACK_CHECK_EN); //0x91 is the slave address hardcoded (the 1 in the LSB tells that it's a read)    
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

float convertTemp (uint8_t data[]) {

    float tempData = 0.0;
    uint8_t temp;

    if (data[0] < 128 ) { //MSB is 0 (positive temp)

        temp = data[0];
        for (uint8_t i = 0; i < 8; i++) {
            tempData += (temp % 2) * myPow(2,i);
            temp /= 2;
        }

        if (data[1] == 128) {
            tempData += 0.5;
        }

    } else { //LSB is 1 (negative temp)
        int16_t bits = 0xff00 | data[0]; //Sign Extended the temp in binary
        bits = bits << 1; //Shift over by one to allow for LSB of Temp
        if (data[1] == 128) { //Adds in the 1 if LSB is 1
            bits += 1;   
        }
        tempData = bits;
        tempData /= 2; 
    }
    return tempData;
}

void app_main(void)
{
//     ESP_LOGI(TAG, "I2C initialized successfully");

// //----------------------------------------------------------------------------------------------------------------------
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();    
    
//     i2c_master_start(cmd);

//     i2c_master_read_byte(cmd, data, false);

//     /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
//     ESP_ERROR_CHECK(i2c_master_read(cmd, data, 1, false));
//     ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / 100);
// //----------------------------------------------------------------------------------------------------------------------
   
    /* Demonstrate writing by reseting the MPU9250 */
    // ESP_ERROR_CHECK(mpu9250_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

//----------------------------------------------------------------------------------------------------------------------
    // ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    // ESP_LOGI(TAG, "I2C de-initialized successfully");

//     uint8_t data_h;
//     uint8_t data_l;
//     i2c_port_t i2c_num = I2C_MASTER_NUM;

//     ESP_ERROR_CHECK(i2c_master_init());

//     vTaskDelay(30 / portTICK_PERIOD_MS);
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, MPU9250_SENSOR_ADDR, ACK_CHECK_EN);
//     i2c_master_read_byte(cmd, &data_h, ACK_VAL);
//     i2c_master_read_byte(cmd, &data_l, NACK_VAL);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
//     ESP_ERROR_CHECK(ret);
//     i2c_cmd_link_delete(cmd);
// //----------------------------------------------------------------------------------------------------------------------
//     ESP_LOGI(TAG, "BH1750 data_h = %X, data_l = %X", data_h, data_l);
    ESP_ERROR_CHECK(i2c_master_init());
    uint8_t IC_data[2];
    float IC_Temp;
    while(1){
        //ESP_ERROR_CHECK(temp_register_read(TEMP_REG, IC_data, 2)); 
        temp_register_read(TEMP_REG, IC_data, 2);
        IC_Temp = convertTemp(IC_data);
        printf("IC Temp = %f ˚C\n",IC_Temp);
    }
}