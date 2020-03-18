#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "tft.h"
#include "tftspi.h"
#include "spiffs_vfs.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#define SPI_BUS TFT_HSPI_HOST

const char* TAG = "IHM";

static char tmp_buff[64];
static time_t time_now, time_last = 0;
static struct tm* tm_now_info;

static const char wday_name[][8] = {
    "dimanche", "lundi", "mardi", "mercredi", "jeudi", "vendredi", "samedi"
};
static const char mon_name[][9] = {
  "janvier", "fevrier", "mars", "avril", "mai", "juin",
  "juillet", "aout", "septembre", "octobre", "novembre", "decembre"
};

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void show_background_image()
{
    TFT_resetclipwin();

	if (spiffs_is_mounted)
	{
	    TFT_jpg_image(CENTER, CENTER, 0, SPIFFS_BASE_PATH"/images/background_sky.jpg", NULL, 0);
	}
}

void show_time()
{
	TFT_resetclipwin();

	if (spiffs_is_mounted)
	{
		ESP_LOGI(TAG, "Showing time");

		tft_font_transparent = 1;
		tft_fg = TFT_WHITE;

		TFT_setFont(USER_FONT, "/spiffs/fonts/DotMatrix_M.fon");
		TFT_print("00:00", 220, 170);
	}
}

void show_date()
{
	TFT_resetclipwin();

	if(spiffs_is_mounted)
	{
		ESP_LOGI(TAG, "Showing date");

		int last_font_height = TFT_getfontheight();
		int last_string_width = TFT_getStringWidth("16:51");

		ESP_LOGI(TAG, "Last font height : %d, LastY : %d", last_font_height, LASTY);

		tft_font_transparent = 1;
		tft_fg = TFT_WHITE;

		TFT_setFont(SMALL_FONT, NULL);

		sprintf(tmp_buff, "%.8s", wday_name[tm_now_info->tm_wday]);
		TFT_print(tmp_buff, 220 + last_string_width/2 - TFT_getStringWidth(tmp_buff)/2, LASTY + last_font_height + 1);

		last_font_height = TFT_getfontheight();
		TFT_setFont(DEF_SMALL_FONT, NULL);

		sprintf(tmp_buff, "%d %s %d", tm_now_info->tm_mday, mon_name[tm_now_info->tm_mon], 1900 + tm_now_info->tm_year );
		TFT_print(tmp_buff, 220 + last_string_width/2 - TFT_getStringWidth(tmp_buff)/2, LASTY + last_font_height + 1);
	}
}


void set_time()
{
	time(&time_now);
	tm_now_info = localtime(&time_now);
}

void vUpdateTimeTask(void * pvParameters)
{
	const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

	tft_fg = TFT_WHITE;
	TFT_setFont(USER_FONT, "/spiffs/fonts/DotMatrix_M.fon");

	for( ;; )
	{
		set_time();

		TFT_setclipwin(220 + TFT_getStringWidth("16"), 170, 220 + TFT_getStringWidth("16:"), 170 + TFT_getfontheight());

		if(tm_now_info->tm_sec == 0 || time_now > (time_last+59))
		{
			time_last = time_now;
			TFT_setclipwin(220, 170, 220 + TFT_getStringWidth("16:51"), 170 + TFT_getfontheight());
		}

		TFT_jpg_image(0, 0, 0, SPIFFS_BASE_PATH"/images/background_sky.jpg", NULL, 0);

		TFT_resetclipwin();

		sprintf(tmp_buff, "%02d %02d", tm_now_info->tm_hour, tm_now_info->tm_min);
		TFT_print(tmp_buff, 220, 170);

		vTaskDelay(xDelay);

		sprintf(tmp_buff, "%02d:%02d", tm_now_info->tm_hour, tm_now_info->tm_min);
		TFT_print(tmp_buff, 220, 170);

		vTaskDelay(xDelay);
	}
}

void init_screen()
{
    esp_err_t ret;
    spi_lobo_device_handle_t spi;

    spi_lobo_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
		.max_transfer_sz = 6*1024,
    };
    spi_lobo_device_interface_config_t devcfg={
        .clock_speed_hz=8000000,
        .mode=0,
        .spics_io_num=-1,
		.spics_ext_io_num=PIN_NUM_CS,
		.flags=LB_SPI_DEVICE_HALFDUPLEX,
    };

    TFT_PinsInit();

    ret = spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi);
    assert(ret==ESP_OK);
    tft_disp_spi = spi;

	ret = spi_lobo_device_select(spi, 1);
    assert(ret==ESP_OK);
	ret = spi_lobo_device_deselect(spi);
    assert(ret==ESP_OK);

    TFT_display_init();

    spi_lobo_set_speed(spi, DEFAULT_SPI_CLOCK);

	tft_font_rotate = 0;
	tft_text_wrap = 0;
	tft_font_transparent = 0;
	tft_font_forceFixed = 0;
	tft_gray_scale = 0;

    TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
	TFT_setRotation(LANDSCAPE);
	TFT_setFont(DEFAULT_FONT, NULL);
	TFT_resetclipwin();

	vfs_spiffs_register();

	vTaskDelay(2000/ portTICK_RATE_MS);

    set_time();

    show_background_image();
    show_time();
    show_date();
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
}

static void obtain_time(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    ESP_ERROR_CHECK(example_connect());

    initialize_sntp();

    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&time_now);
    localtime_r(&time_now, tm_now_info);

    ESP_ERROR_CHECK( example_disconnect() );
}


void init_wifi()
{
    set_time();

    if (tm_now_info->tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();

        time(&time_now);

        setenv("TZ", "UTC-1", 1);
        tzset();
    }
}

void app_main(void)
{
    static uint8_t ucParameterToPass = 1;
    TaskHandle_t xHandle = NULL;

    init_wifi();
    init_screen();

    xTaskCreate( vUpdateTimeTask, "update time", 8192, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle );
    //xTaskCreate( vUpdateTimeMinutesTask, "update time", 8192, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle );
    //xTaskCreate( vUpdateTimeMinutesTask, "update time", 8192, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle );
    //configASSERT( xHandle );

}

