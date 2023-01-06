/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "driver/i2c.h"

#include "esp_err.h"

#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_interface.h"

#include "esp_lcd_touch.h"
#include "esp_lcd_touch_gt911.h"

#include "esp_timer.h"

#include "esp_heap_caps.h"

#include "lvgl.h"
#include "demos/widgets/lv_demo_widgets.h"
#include "demos/benchmark/lv_demo_benchmark.h"
//----------------------------------------------------------------------------

static const char *TAG = "example";

#define	LCD_BL		GPIO_NUM_2	// Back Light

#define	LCD_DE		GPIO_NUM_40	/* DE 	 */
#define	LCD_VSYNC	GPIO_NUM_41 /* VSYNC */
#define	LCD_HSYNC	GPIO_NUM_39 /* HSYNC */
#define LCD_PCLK	GPIO_NUM_42 /* PCLK  */

#define LCD_R0		GPIO_NUM_45	/* R0 */
#define LCD_R1		GPIO_NUM_48	/* R1 */
#define LCD_R2		GPIO_NUM_47	/* R2 */
#define LCD_R3		GPIO_NUM_21	/* R3 */
#define LCD_R4		GPIO_NUM_14	/* R4 */

#define LCD_G0		GPIO_NUM_5	/* G0 */
#define LCD_G1		GPIO_NUM_6	/* G1 */
#define LCD_G2		GPIO_NUM_7	/* G2 */
#define LCD_G3		GPIO_NUM_15	/* G3 */
#define LCD_G4		GPIO_NUM_16	/* G4 */
#define LCD_G5		GPIO_NUM_4	/* G5 */

#define LCD_B0		GPIO_NUM_8	/* B0 */
#define LCD_B1		GPIO_NUM_3	/* B1 */
#define LCD_B2		GPIO_NUM_46	/* B2 */
#define LCD_B3		GPIO_NUM_9	/* B3 */
#define LCD_B4		GPIO_NUM_1	/* B4 */
//----------------------------------------------------------------------------

static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) disp_drv->user_data;
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);

    lv_disp_flush_ready(disp_drv);
}

static void lv_tick_inc_cb(void *data)
{
    uint32_t tick_inc_period_ms = *((uint32_t *) data);

    lv_tick_inc(tick_inc_period_ms);
}

esp_err_t lv_port_tick_init(void)
{
    static uint32_t tick_inc_period_ms = 5;
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = lv_tick_inc_cb,
            .arg = &tick_inc_period_ms,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "",     /* name is optional, but may help identify the timer when debugging */
            .skip_unhandled_events = true,
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet. Start the timer now */
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, tick_inc_period_ms * 1000));

    return ESP_OK;
}

#define GT911_SCL 		GPIO_NUM_20
#define GT911_SDA 		GPIO_NUM_19
#define GT911_INT 		GPIO_NUM_NC
#define GT911_RST 		GPIO_NUM_38
#define GT911_WIDTH		800
#define GT911_HEIGHT	480
#define	GT911_I2C_SPEED	(400 * 1000)

static esp_lcd_touch_handle_t tp;   // LCD touch handle
static void touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t) indev_drv->user_data;
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    assert(tp);


    /* Read data from touch controller into memory */
    esp_lcd_touch_read_data(tp);

    /* Read data from touch controller */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static esp_err_t lvgl_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv_tp;
    lv_indev_t *indev_touchpad;

	const i2c_config_t i2c_conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = GT911_SDA,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = GT911_SCL,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = GT911_I2C_SPEED
	};
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	vTaskDelay(100 / portTICK_PERIOD_MS);

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &tp_io_config, &tp_io_handle));

    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = GT911_WIDTH,
        .y_max = GT911_HEIGHT,
        .rst_gpio_num = GT911_RST,
        .int_gpio_num = GT911_INT,
        .levels = {
            .reset = 1,
            .interrupt = 1,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));

    /* Register a touch input device */
    lv_indev_drv_init(&indev_drv_tp);
    indev_drv_tp.type = LV_INDEV_TYPE_POINTER;
    indev_drv_tp.read_cb = touchpad_read;
    indev_drv_tp.user_data = tp;
    indev_touchpad = lv_indev_drv_register(&indev_drv_tp);
    ESP_ERROR_CHECK((indev_touchpad == NULL ? ESP_ERR_NO_MEM : ESP_OK));

    return ESP_OK;
}

void write_log(const char *format, ...) {
	va_list args;
	va_start(args, format);
	vprintf(format, args);
	va_end(args);
}


void app_main(void)
{
	static lv_disp_draw_buf_t draw_buf;

//	lv_color_t *buf1 = heap_caps_aligned_calloc(64, 1,
//			800 * 480 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
//	ESP_ERROR_CHECK((buf1 == NULL ? ESP_ERR_NO_MEM : ESP_OK));
//
//	lv_color_t *buf2 = heap_caps_aligned_calloc(64, 1,
//			800 * 480 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
//	ESP_ERROR_CHECK((buf2 == NULL ? ESP_ERR_NO_MEM : ESP_OK));

	lv_color_t *buf1 = heap_caps_aligned_calloc(8, 1,
			800 * 80 * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
	ESP_ERROR_CHECK((buf1 == NULL ? ESP_ERR_NO_MEM : ESP_OK));

	esp_lcd_rgb_panel_config_t rgb_panel_config;
	esp_lcd_panel_handle_t ret_panel;

	gpio_set_direction(LCD_BL, GPIO_MODE_OUTPUT);
	gpio_set_level(LCD_BL, 0);

	memset(&rgb_panel_config, 0, sizeof(rgb_panel_config));

	rgb_panel_config.data_width = 16;
	rgb_panel_config.sram_trans_align = 8;
	rgb_panel_config.psram_trans_align = 64;

	rgb_panel_config.clk_src = LCD_CLK_SRC_DEFAULT;
	rgb_panel_config.de_gpio_num = LCD_DE;
	rgb_panel_config.pclk_gpio_num = LCD_PCLK;
	rgb_panel_config.hsync_gpio_num = LCD_HSYNC;
	rgb_panel_config.vsync_gpio_num = LCD_VSYNC;

	rgb_panel_config.bounce_buffer_size_px = 800 * 8;

	rgb_panel_config.data_gpio_nums[0] = LCD_B0;
	rgb_panel_config.data_gpio_nums[1] = LCD_B1;
	rgb_panel_config.data_gpio_nums[2] = LCD_B2;
	rgb_panel_config.data_gpio_nums[3] = LCD_B3;
	rgb_panel_config.data_gpio_nums[4] = LCD_B4;
	rgb_panel_config.data_gpio_nums[5] = LCD_G0;
	rgb_panel_config.data_gpio_nums[6] = LCD_G1;
	rgb_panel_config.data_gpio_nums[7] = LCD_G2;
	rgb_panel_config.data_gpio_nums[8] = LCD_G3;
	rgb_panel_config.data_gpio_nums[9] = LCD_G4;
	rgb_panel_config.data_gpio_nums[10] = LCD_G5;
	rgb_panel_config.data_gpio_nums[11] = LCD_R0;
	rgb_panel_config.data_gpio_nums[12] = LCD_R1;
	rgb_panel_config.data_gpio_nums[13] = LCD_R2;
	rgb_panel_config.data_gpio_nums[14] = LCD_R3;
	rgb_panel_config.data_gpio_nums[15] = LCD_R4;

	rgb_panel_config.timings.flags.pclk_idle_high = 0;
	rgb_panel_config.timings.flags.de_idle_high = 0;
	rgb_panel_config.timings.flags.pclk_active_neg = 1;
	rgb_panel_config.timings.pclk_hz = 20 * 1000 * 1000;
	rgb_panel_config.timings.h_res = 800;
	rgb_panel_config.timings.hsync_back_porch = 8;
	rgb_panel_config.timings.hsync_front_porch = 8;
	rgb_panel_config.timings.hsync_pulse_width = 4;
	rgb_panel_config.timings.v_res = 480;
	rgb_panel_config.timings.vsync_back_porch = 8;
	rgb_panel_config.timings.vsync_front_porch = 8;
	rgb_panel_config.timings.vsync_pulse_width = 4;

	rgb_panel_config.disp_gpio_num = GPIO_NUM_NC;

	rgb_panel_config.flags.fb_in_psram = 1;
	rgb_panel_config.flags.disp_active_low = 0;
	rgb_panel_config.flags.double_fb = 0;

	ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&rgb_panel_config, &ret_panel));

	ESP_ERROR_CHECK(esp_lcd_panel_reset(ret_panel));
	ESP_ERROR_CHECK(esp_lcd_panel_init(ret_panel));

	ESP_LOGI(TAG, "LCD_RGB_PANEL initialized OK...");

	lv_init();

	lv_disp_draw_buf_init(&draw_buf, buf1, NULL, 800 * 80);

	lv_disp_drv_t driver;
	lv_disp_drv_init(&driver);
	driver.hor_res = 800;
	driver.ver_res = 480;
	driver.user_data = ret_panel;

	driver.flush_cb = disp_flush;
	driver.draw_buf = &draw_buf;

	lv_disp_drv_register(&driver);

	lvgl_port_indev_init();

	lv_demo_widgets();

	//lv_demo_benchmark(LV_DEMO_BENCHMARK_MODE_RENDER_AND_DRIVER);

	gpio_set_level(LCD_BL, 1);

	lv_port_tick_init();

	write_log(LOG_COLOR(LOG_COLOR_GREEN)"\r\nFree memory MALLOC_CAP_DEFAULT\r\n"LOG_RESET_COLOR);
	heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
	write_log(LOG_COLOR(LOG_COLOR_GREEN)"\r\nFree memory MALLOC_CAP_INTERNAL\r\n"LOG_RESET_COLOR);
	heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);

	write_log(LOG_COLOR(LOG_COLOR_GREEN)"\r\nApplication is running...\r\n"LOG_RESET_COLOR);

    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);

        lv_task_handler();
    }
}
