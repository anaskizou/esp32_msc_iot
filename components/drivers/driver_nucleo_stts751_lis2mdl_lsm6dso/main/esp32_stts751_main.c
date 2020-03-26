/* i2c - LIS2MDL and LSM6DSO for ESP 32
   Adapted from ESPRESSIF example
   LAGHMAMI AYMANE 03/2020

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "stts751_reg.h"
#include "lis2mdl_reg.h"
#include "lsm6dso_reg.h"
#include "lis2mdl_reg.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

static const char *TAG = "esp32_lsm6dso_lis2mdl";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define LIS2MDL_ADDR_7BITS 0x1e
#define LSM6DSO_ADDR_7BITS 0x6b
#define SENSOR_BUS I2C_MASTER_NUM
#define SENSOR_BUS2 I2C_MASTER_NUM
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define WRITE_BIT2 I2C2_MASTER_WRITE              /*!< I2C master write */

#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis1bit16_t data_raw_temperature;
static axis3bit16_t data_raw_magnetic;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_mG[3];
static float temperature_degC;

stmdev_ctx_t dev_ctx;
stmdev_ctx_t dev2_ctx;
static uint8_t whoamI, rst;
static uint8_t whoamI2, rst2;
TaskHandle_t xHandle = NULL;


/**
 * @brief test code to read esp-i2c-slave like a memory device.
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _____________________________________________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte (register address) + ack | start | slave_addr + rd_bit + ack | read n bytes + nack | stop |
 * --------|---------------------------|---------------------------------------|-------|---------------------------|---------------------|------|
 *
 */
static int32_t i2c_master_read_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSO_ADDR_7BITS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSO_ADDR_7BITS<< 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static int32_t i2c2_master_read_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS2MDL_ADDR_7BITS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS2MDL_ADDR_7BITS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave memory like device,
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * __________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte (register address) + ack | write n bytes + ack  | stop |
 * --------|---------------------------|---------------------------------------|----------------------|------|
 *
 */
static int32_t i2c_master_write_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_wr, uint16_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSO_ADDR_7BITS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
static int32_t i2c2_master_write_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_wr, uint16_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS2MDL_ADDR_7BITS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE; // Pullup resistors are already present on X-NUCLEO-IKS01A3
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void get_gyro_task(void *args)
{
	/* Read output only if not busy */
	uint8_t reg;

 while (1) {
	lsm6dso_gy_flag_data_ready_get(&dev_ctx, &reg);
	if (reg)
	{
      /* Read angular rate field data*/

      memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
      lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
      angular_rate_mdps[0] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
	  angular_rate_mdps[1] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
	  angular_rate_mdps[2] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);

	  printf("Angular rate [mdps]:%4.2f\t  %4.2f\t  %4.2f\r\n",angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
	  vTaskDelay(4000 / portTICK_PERIOD_MS);
	}

   }
}

static void get_accelo_task(void *args)
{
	/* Read output only if not busy */
	uint8_t reg;

 while (1) {
	lsm6dso_xl_flag_data_ready_get(&dev_ctx, &reg);
	if (reg)
	{
      /* Read acceleration field data */
      memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
      lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
      acceleration_mg[0] = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);

	  printf("Acceleration [mg]:%4.2f\t  %4.2f\t  %4.2f\r\n",acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
	  vTaskDelay(4000 / portTICK_PERIOD_MS);
	}
   }
}

static void get_magn_task(void *args)
{
	/* Read output only if not busy */
	uint8_t reg;

 while (1) {
	 lis2mdl_mag_data_ready_get(&dev2_ctx, &reg);
	if (reg)
	{
      /* Read angular rate field data*/

	  memset(data_raw_magnetic.u8bit, 0x00, 3 * sizeof(int16_t));
	  lis2mdl_magnetic_raw_get(&dev2_ctx, data_raw_magnetic.u8bit);
	  magnetic_mG[0] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[0]);
	  magnetic_mG[1] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[1]);
	  magnetic_mG[2] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[2]);

	  printf("Magnetic field [mG]:%4.2f\t  %4.2f\t  %4.2f\r\n",magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
	  vTaskDelay(4000 / portTICK_PERIOD_MS);
	}

   }
}


static void whoami_task(void *args)
{
    int ret;

    while (1) {
        ESP_LOGI(TAG, "WHOAMI TASK");

        lsm6dso_device_id_get(&dev_ctx, &whoamI);
        lis2mdl_device_id_get(&dev2_ctx, &whoamI2);
        if ((whoamI != LIS2MDL_ID)&&(whoamI2 != LSM6DSO_ID))
        ret = ESP_ERR_TIMEOUT; /* manage here device not found */
        else ret = ESP_OK;
       
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("************************************\n");
            printf("WHOAMI MASTER CHECK SENSOR (LIS2MDL & LSM6DSO )\n");
            printf("************************************\n");
        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }

       printf("End of Task WHOAMI!\n\n");

       /* whoami task is run only once. At the end we start get the different data task n*/
       xTaskCreate(get_magn_task, "lis2mdl_get_magn_task", 1024 * 2, (void *)0, 5, NULL);
       xTaskCreate(get_gyro_task, "lsm6dso_get_temp_task", 1024 * 2, (void *)0, 5, NULL);
       xTaskCreate(get_accelo_task, "lsm6dso_get_accelo_task", 1024 * 2, (void *)0, 5, NULL);
       vTaskDelete(xHandle);
  }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    /* This acts as the entry point of ST's STTS751 driver */
    dev_ctx.write_reg = i2c_master_write_slave;
    dev_ctx.read_reg = i2c_master_read_slave;
    dev_ctx.i2c_number = SENSOR_BUS;

    dev2_ctx.write_reg = i2c2_master_write_slave;
    dev2_ctx.read_reg = i2c2_master_read_slave;
    dev2_ctx.i2c_number = SENSOR_BUS2;

    /* Enable interrupt on high(=49.5 degC)/low(=-4.5 degC) temperature. */
    float temperature_high_limit = 49.5f;
    stts751_high_temperature_threshold_set(&dev_ctx, stts751_from_celsius_to_lsb(temperature_high_limit));

    float temperature_low_limit = -4.5f;
    stts751_low_temperature_threshold_set(&dev_ctx, stts751_from_celsius_to_lsb(temperature_low_limit));

    stts751_pin_event_route_set(&dev_ctx,  PROPERTY_ENABLE);

    /* Disable I3C interface */
    lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);

    /* Enable Block Data Update */
    lis2mdl_block_data_update_set(&dev2_ctx, PROPERTY_ENABLE);
    lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

      /* Set Output Data Rate */
    lis2mdl_data_rate_set(&dev2_ctx, LIS2MDL_ODR_10Hz);
    stts751_temp_data_rate_set(&dev_ctx, STTS751_TEMP_ODR_250mHz);
    lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_12Hz5);
    lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_12Hz5);

    /* Set / Reset sensor mode */
    lis2mdl_set_rst_mode_set(&dev2_ctx, LIS2MDL_SENS_OFF_CANC_EVERY_ODR);

    /* Enable temperature compensation */
    lis2mdl_offset_temp_comp_set(&dev2_ctx, PROPERTY_ENABLE);
    lis2mdl_operating_mode_set(&dev2_ctx, LIS2MDL_CONTINUOUS_MODE);

    /*Accelerometer - LPF1 + LPF2 path*/
    lsm6dso_xl_hp_path_on_out_set(&dev_ctx, LSM6DSO_LP_ODR_DIV_100);
    lsm6dso_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

    /* Set Full Scale */
    lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
    lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps);

    /* Set Resolution */
    stts751_resolution_set(&dev_ctx, STTS751_11bit);

    /* start whoami task */
    xTaskCreate(whoami_task, "i2c_whoami_task", 1024 * 2, (void *)0, 10, &xHandle);
}
