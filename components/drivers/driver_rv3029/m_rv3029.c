
#include "m_rv3029.h"

#define BCD2BIN(val) ((((val) & 0x0f) + ((val) >> 4) * 10))
#define BIN2BCD(val) ((((val)/10)<<4) + (val)%10)

int rv3029_init(i2c_port_t port, int sda_pin, int scl_pin,  gpio_pullup_t sda_internal_pullup,  gpio_pullup_t scl_internal_pullup) {
	
	esp_err_t ret;
	_port = port;
	
	
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = sda_pin;
	conf.scl_io_num = scl_pin;
	conf.sda_pullup_en = sda_internal_pullup;
	conf.scl_pullup_en = scl_internal_pullup;
	conf.master.clk_speed = 100000;
	ret = i2c_param_config(port, &conf);
	if(ret != ESP_OK) return RV3029_ERR_CONFIG;
	
	
	ret = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
	if(ret != ESP_OK) return RV3029_ERR_INSTALL;
	
	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (RV3029_ADDR << 1) | I2C_MASTER_WRITE, true);
	i2c_master_stop(cmd);
	if(i2c_master_cmd_begin(_port, cmd, 1000 / portTICK_RATE_MS) != ESP_OK)
		return RV3029_ERR_NOTFOUND;
	else return RV3029_ERR_OK;
}

int8_t rv3029_temp(void)

{
  esp_err_t ret;
  uint8_t temp;
  
  ret = rv3029_read(0x20, 1, &temp);

  if (ret == ESP_OK) return (int8_t)temp - 60;
  else return RV3029_ERR_TIMEOUT;
}

int rv3029_read(uint8_t regaddr, uint8_t bytes, uint8_t *val) 
{
   esp_err_t ret;
   
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (RV3029_ADDR << 1) | I2C_MASTER_WRITE, true);
   i2c_master_write_byte(cmd, regaddr, true);
  
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (RV3029_ADDR << 1) | I2C_MASTER_READ, true);
   
   if (bytes > 1) {
        i2c_master_read(cmd, val, bytes - 1, I2C_MASTER_ACK);
    }
   i2c_master_read_byte(cmd, val + bytes - 1, I2C_MASTER_NACK);
   
   i2c_master_stop(cmd);
   ret = i2c_master_cmd_begin(_port, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   
   if(ret != ESP_OK) return RV3029_ERR_TIMEOUT;
   else return RV3029_ERR_OK;
}

int rv3029_write(uint8_t regaddr, uint8_t bytes, uint8_t *val)
{
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RV3029_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, regaddr, true);
    i2c_master_write(cmd, val, bytes, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    if(ret != ESP_OK) return RV3029_ERR_TIMEOUT;
    else return RV3029_ERR_OK;
}

		
int rv3029_get_time(struct TimeSpec *time) {
	
	uint8_t *raw = (uint8_t *)malloc(7);
	
	if (rv3029_read(0x08, 7, raw) != RV3029_ERR_OK) return RV3029_ERR_TIMEOUT;
	
	time->seconds = BCD2BIN(raw[0]);
	time->minutes = BCD2BIN(raw[1]);
	time->hours = BCD2BIN(raw[2]);
	time->date = BCD2BIN(raw[3]);
	time->dow = raw[4];
	time->months = BCD2BIN(raw[5]);
	time->years = BCD2BIN(raw[6]);

	return RV3029_ERR_OK;
}

int rv3029_set_time(struct TimeSpec *time) {
	
	uint8_t *raw = (uint8_t *)malloc(7);
	
	raw[0] = BIN2BCD(time->seconds);
	raw[1] = BIN2BCD(time->minutes);
	raw[2] = BIN2BCD(time->hours);
	raw[3] = BIN2BCD(time->date);
	raw[4] = time->dow;
	raw[5] = BIN2BCD(time->months);
	raw[6] = BIN2BCD(time->years);
	
	return rv3029_write(0x08, 7, raw);
}
