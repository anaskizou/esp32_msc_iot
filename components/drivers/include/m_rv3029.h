 
 
#include "esp_err.h"
 




#include "freertos/task.h"
#include "driver/i2c.h"
 
#ifndef __ESP_RV3029_H__
#define __ESP_RV3029_H__

// RTC address
#define RV3029_ADDR	0x56


#define RV3029_ERR_OK				0x00
#define RV3029_ERR_CONFIG			0x01
#define RV3029_ERR_INSTALL			0x02
#define RV3029_ERR_NOTFOUND			0x03
#define RV3029_ERR_INVALID_ARG		        0x04
#define RV3029_ERR_FAIL		 		0x05
#define RV3029_ERR_INVALID_STATE	        0x06
#define RV3029_ERR_TIMEOUT	                0x07

// variables
i2c_port_t _port;


struct TimeSpec {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t date;
	uint8_t dow;
	uint8_t months;
	uint8_t years;
};

// functions
int rv3029_init(i2c_port_t port, int sda_pin, int scl_pin, gpio_pullup_t sda_internal_pullup, gpio_pullup_t scl_internal_pullup);
int rv3029_read(uint8_t regaddr, uint8_t bytes, uint8_t *val);
int rv3029_write(uint8_t regaddr, uint8_t bytes, uint8_t *val);
int8_t rv3029_temp(void);
int rv3029_get_time(struct TimeSpec *time);
int rv3029_set_time(struct TimeSpec *time);

#endif // __ESP_RV3029_H__
