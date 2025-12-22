#pragma once

#include "mqtt_client.h"
#include "esp_log.h"
#include "dl_model_base.hpp"
#include "dl_image_define.hpp"
#include "dl_image_preprocessor.hpp"
#include "dl_cls_postprocessor.hpp"

// MQTT Configuration - UPDATE THESE 
#define MQTT_BROKER_URI "mqtts://c62cdd8a93904d22b9667ddb4e1f0760.s1.eu.hivemq.cloud:8883"
#define MQTT_USERNAME   "tinybirdiary"
#define MQTT_PASSWORD   "TinyBirdiary2024"

namespace mqtt {
void send_image(const uint8_t* jpeg_data, size_t jpeg_size);
void send_classification(const dl::cls::result_t &classification);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void* event_data);
void mqtt_app_start();
} // namespace mqtt