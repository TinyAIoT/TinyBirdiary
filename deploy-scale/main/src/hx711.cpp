#include "hx711.hpp"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <climits>

static const char *TAG = "HX711";

static int s_data_pin = -1;
static int s_clk_pin  = -1;
static long s_tare_offset = 0;

void hx711_init(int data_pin, int clk_pin) {
    s_data_pin = data_pin;
    s_clk_pin  = clk_pin;

    gpio_config_t io_conf = {};

    // SCK: output, idle low
    io_conf.pin_bit_mask = (1ULL << clk_pin);
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)clk_pin, 0);

    // DOUT: input
    io_conf.pin_bit_mask = (1ULL << data_pin);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Initialized: DOUT=GPIO%d SCK=GPIO%d", data_pin, clk_pin);
}

bool hx711_is_ready() {
    return gpio_get_level((gpio_num_t)s_data_pin) == 0;
}

// Read one 24-bit two's-complement sample, Channel A, gain 128
static long hx711_read_raw() {
    // Wait for conversion (DOUT goes low), max ~200 ms
    int timeout_ms = 200;
    while (!hx711_is_ready() && timeout_ms-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (timeout_ms <= 0) {
        ESP_LOGW(TAG, "Timeout waiting for HX711 ready");
        return LONG_MIN; // sentinel: invalid sample
    }

    long data = 0;
    // 24 clock pulses to shift out the data, MSB first
    for (int i = 0; i < 24; i++) {
        gpio_set_level((gpio_num_t)s_clk_pin, 1);
        ets_delay_us(1);
        data = (data << 1) | gpio_get_level((gpio_num_t)s_data_pin);
        gpio_set_level((gpio_num_t)s_clk_pin, 0);
        ets_delay_us(1);
    }

    // 25th pulse: select Channel A, Gain 128 for next conversion
    gpio_set_level((gpio_num_t)s_clk_pin, 1);
    ets_delay_us(1);
    gpio_set_level((gpio_num_t)s_clk_pin, 0);
    ets_delay_us(1);

    // Sign-extend 24-bit two's complement to 32-bit
    if (data & 0x800000) {
        data |= 0xFF000000;
    }

    return data;
}

void hx711_tare(int samples) {
    long sum = 0;
    int valid = 0;
    int attempts = 0;
    const int max_attempts = samples * 5; // retry up to 5x per sample
    while (valid < samples && attempts < max_attempts) {
        long raw = hx711_read_raw();
        attempts++;
        if (raw == LONG_MIN) {
            ESP_LOGW(TAG, "Tare: skipping timeout read (%d/%d valid)", valid, samples);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        sum += raw;
        valid++;
    }
    if (valid == 0) {
        ESP_LOGE(TAG, "Tare failed: no valid readings after %d attempts", attempts);
        return;
    }
    s_tare_offset = sum / valid;
    ESP_LOGI(TAG, "Tare complete: offset=%ld (from %d/%d samples)", s_tare_offset, valid, samples);
}

float hx711_read_grams() {
    long raw = hx711_read_raw();
    if (raw == LONG_MIN) return 0.0f; // timeout: treat as no change
    float grams = (float)(raw - s_tare_offset) / HX711_CALIBRATION_FACTOR;
    ESP_LOGD(TAG, "raw=%ld tare=%ld grams=%.2f", raw, s_tare_offset, grams);
    return grams;
}
