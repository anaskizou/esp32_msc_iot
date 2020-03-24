
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "m_rv3029.h"
#include "m_rv3029.c"
#include "sdkconfig.h"


#define I2C_SDA_PIN CONFIG_I2C_SDA_PIN
#define I2C_SCL_PIN CONFIG_I2C_SCL_PIN

void app_main()
{
  int8_t temperature;
  
  if (rv3029_init(I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN, GPIO_PULLUP_DISABLE, GPIO_PULLUP_DISABLE) != RV3029_ERR_OK)
	  printf("No RV3029 found!\r\n");
  else 
	  printf("RV3029 found \r\n");

  temperature = rv3029_temp();
  
  if (temperature)
     printf("Temperature value = %d\r\n",temperature);   
  else
     printf("There was a problem!\r\n");
     
  vTaskDelay(1000 / portTICK_PERIOD_MS); 
  
  struct TimeSpec mytime;
  
  mytime.years = 20;
  mytime.months = 3;
  mytime.date = 11;
  mytime.dow = 3;
  mytime.hours = 19;
  mytime.minutes = 28;
  mytime.seconds = 0;
  
  if (rv3029_set_time(&mytime) != RV3029_ERR_OK)
      printf("problem setting time\r\n");
  else printf("Write OK:\r\n");
     
  struct TimeSpec time;
  if (rv3029_get_time(&time) != RV3029_ERR_OK) {
      printf("problem getting time\r\n");
    }
  else {
      printf("Heure Minutes Secondes = %2d:%02d:%02d\r\n", time.hours, time.minutes, time.seconds);
      printf("Jour de la semaine = %d\r\n",time.dow);
      printf("Date = %d-%02d-%02d\r\n", 2000 + time.years, time.months, time.date);
    }
  
}
