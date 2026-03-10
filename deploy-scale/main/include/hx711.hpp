#pragma once
#include <stdbool.h>

// Calibration factor: raw ADC units per gram.
// To calibrate: flash with 1.0f, place a known weight, read raw_value from log,
// then set CALIBRATION_FACTOR = raw_value / known_weight_grams.
#define HX711_CALIBRATION_FACTOR 1000.0f

void hx711_init(int data_pin, int clk_pin);
void hx711_tare(int samples = 10);
float hx711_read_grams();
bool hx711_is_ready();
