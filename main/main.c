#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_heap_caps.h"

#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#define LCD_H_RES          720
#define LCD_V_RES          720
#define LCD_PIXEL_CLOCK_HZ (16 * 1000 * 1000)  // Safe bring-up clock

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    40
#define I2C_SCL_GPIO    41
#define PCA9554_ADDR    0x38

static esp_lcd_panel_handle_t rgb_panel_init(uint16_t *framebuffer)
{
    ESP_LOGI(__func__, "Initializing RGB panel");

    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,

        .timings = {
            .pclk_hz = LCD_PIXEL_CLOCK_HZ,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES,

            .hsync_pulse_width = 10,
            .hsync_back_porch  = 50,
            .hsync_front_porch = 50,

            .vsync_pulse_width = 10,
            .vsync_back_porch  = 20,
            .vsync_front_porch = 20,

            .flags = {
                .pclk_active_neg = false,
                .de_idle_high    = false,
            },
        },

        .data_width = 16,
        .bits_per_pixel = 16,
        .num_fbs = 1,

        .psram_trans_align = 64,
        .sram_trans_align  = 64,

        .disp_gpio_num  = 38, // DE
        .pclk_gpio_num  = 45,
        .vsync_gpio_num = 48,
        .hsync_gpio_num = 47,

        // Only 16 entries allowed in ESP-IDF v5.4.2
        .data_gpio_nums = {
            // R0-R5
            4, 5, 6, 7, 15, 16,
            // G0-G5
            17, 18, 8, 3, 46, 9,
            // B0-B3 (lower 4 bits only)
            10, 11, 12, 13
        },

        .flags = {
            .fb_in_psram = true,
        },
    };

    esp_lcd_panel_handle_t panel;
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));

    ESP_LOGI(__func__, "RGB panel ready");
    return panel;
}

static void panel_reset_release(void)
{
    ESP_LOGI(__func__, "Releasing panel reset via PCA9554 IO2");

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));

    // Install driver only once
    esp_err_t err = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    // 1) Set all pins as outputs
    uint8_t cfg[] = { 0x03, 0x00 };
    ESP_ERROR_CHECK(i2c_master_write_to_device(
        I2C_PORT, PCA9554_ADDR, cfg, sizeof(cfg), pdMS_TO_TICKS(100)));

    // 2) Assert reset (IO2 = 0)
    uint8_t out_assert[] = { 0x01, 0x00 };
    ESP_ERROR_CHECK(i2c_master_write_to_device(
        I2C_PORT, PCA9554_ADDR, out_assert, sizeof(out_assert), pdMS_TO_TICKS(100)));

    vTaskDelay(pdMS_TO_TICKS(10));

    // 3) Release reset (IO2 = 1)
    uint8_t out_release[] = { 0x01, 0x04 }; // bit 2 high
    ESP_ERROR_CHECK(i2c_master_write_to_device(
        I2C_PORT, PCA9554_ADDR, out_release, sizeof(out_release), pdMS_TO_TICKS(100)));

    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(__func__, "Panel reset released");
}



static void backlight_on(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << 2,   // GPIO2
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(2, 1);  // Enable backlight
    ESP_LOGI(__func__, "Backlight enabled");
}

void app_main(void)
{
    ESP_LOGI(__func__, "app_main started");

    size_t fb_size = LCD_H_RES * LCD_V_RES * 2;
    uint16_t *framebuffer = heap_caps_malloc(
        fb_size,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );

    if (!framebuffer) {
        ESP_LOGE(__func__, "Framebuffer allocation failed");
        return;
    }

    ESP_LOGI(__func__, "Framebuffer allocated (%u bytes)", fb_size);

    // Solid red
    for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
        framebuffer[i] = 0xF800;
    }

    panel_reset_release();     // 🔴 MUST be first
    backlight_on();
    vTaskDelay(pdMS_TO_TICKS(20));

    esp_lcd_panel_handle_t panel = rgb_panel_init(framebuffer);

    ESP_ERROR_CHECK(
        esp_lcd_panel_draw_bitmap(
            panel,
            0, 0,
            LCD_H_RES,
            LCD_V_RES,
            framebuffer
        )
    );

    ESP_LOGI(__func__, "Frame drawn");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

