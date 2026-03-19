#include "esp_log.h"
#include "dl_model_base.hpp"
#include "dl_image_define.hpp"
#include "dl_image_preprocessor.hpp"
#include "dl_cls_postprocessor.hpp"
#include "dl_image_jpeg.hpp"
#include "bsp/esp-bsp.h"
#include <esp_system.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "include/ClassificationPostProcessor.hpp"
#include "include/classification_category_name.hpp"
#include "esp_camera.h"

#include "sd_card.hpp"
#include "mqtt.hpp"

#include <dirent.h>
#include <vector>
#include <limits>
#include <map>
#include <string>

#include "include/camera_pins.h"
#include "include/hx711.hpp"

#define HX711_DATA_PIN           14
#define HX711_CLK_PIN            48
#define HX711_WEIGHT_THRESHOLD_G 6.0f

#define MAX_CAPTURE_IMAGES  200
#define MIN_CAPTURE_IMAGES  50
#define CAPTURE_INTERVAL_MS 50

// WiFi includes
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "mbedtls/base64.h"

#include "esp_sntp.h"
#include <time.h>

// WiFi Configuration - UPDATE THESE WITH YOUR CREDENTIALS
#define WIFI_SSID      "XXX"
#define WIFI_PASSWORD      "XXX"
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
static const char *TAG = "APP";

#if CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

// WiFi event group bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// Support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

extern const uint8_t espdl_model[] asm("_binary_birdiary_v5_squeezenet_layerwise_equalization_2_10_2_espdl_start");
static const char *model_path = (const char *)espdl_model;

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

const char *out_dir = "/sdcard/classification";
char background_log_path[128];

// Classification model (loaded once at startup)
static dl::Model *s_model = nullptr;
static dl::image::ImagePreprocessor *s_preprocessor = nullptr;

// Capture session shared state
#define CAPTURE_DONE_BIT  BIT0
#define CLASSIFY_DONE_BIT BIT1
#define WEIGHT_CONFIRMED_BIT BIT2
#define WEIGHT_REJECTED_BIT  BIT3

struct CaptureSession {
    dl::image::jpeg_img_t images[MAX_CAPTURE_IMAGES];
    volatile int image_count;
    float initial_weight_g;
    EventGroupHandle_t done_event;
};
static CaptureSession s_session;

static void obtain_time(void)
{
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;

    while (timeinfo.tm_year < (2016 - 1900) && ++retry < 20) {
        vTaskDelay(pdMS_TO_TICKS(500));
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    // Optional: set TZ for Germany
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();
}

static void event_handler(void* arg, const char* event_base,
                                long int event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI("Wifi", "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI("Wifi","connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("Wifi", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold = {
                .authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD
            },
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
#ifdef CONFIG_ESP_WIFI_WPA3_COMPATIBLE_SUPPORT
            .disable_wpa3_compatible_mode = 0,
#endif
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASSWORD);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static const dl::cls::result_t run_inference(dl::Model *model, dl::image::ImagePreprocessor *m_image_preprocessor, dl::image::img_t &input_img) {
    uint32_t t0, t1;
    float delta;
    t0 = esp_timer_get_time();

    m_image_preprocessor->preprocess(input_img);

    model->run(dl::RUNTIME_MODE_MULTI_CORE);
    const int check = 5;
    ClassificationPostProcessor m_postprocessor(model, check, std::numeric_limits<float>::lowest(), true);
    std::vector<dl::cls::result_t> &results = m_postprocessor.postprocess();

    t1 = esp_timer_get_time();
    delta = t1 - t0;
    printf("Inference in %8.0f us.\n", delta);

    dl::cls::result_t best_result = {};
    bool found_result = false;

    for (auto &res : results) {
        ESP_LOGI("CLS", "category: %s, score: %f\n", res.cat_name, res.score);
        if (!found_result || res.score > best_result.score) {
            best_result = res;
            found_result = true;
        }
    }

    return best_result;
}

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,

    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,

    .jpeg_quality = 8,
    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = 0
};

static esp_err_t init_camera(void) {
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE("CAM", "Camera Init Failed");
    }
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);   // 1 = flip vertically, 0 = normal

    return err;
}

static bool capture_image(dl::image::jpeg_img_t &output_img) {
    ESP_LOGI("CAM", "Taking picture...");
    camera_fb_t *pic = esp_camera_fb_get();
    if (!pic) {
        ESP_LOGE("CAM", "Failed to capture image");
        return false;
    }

    ESP_LOGI("CAM", "Picture taken! Its size was: %zu bytes", pic->len);
    ESP_LOGW("image_dim", "Height: %d, Width: %d, Len: %zu", pic->height, pic->width, pic->len);

    output_img.height = pic->height;
    output_img.width = pic->width;
    output_img.data_size = pic->len;

    output_img.data = static_cast<uint8_t*>(malloc(pic->len));
    if (!output_img.data) {
        ESP_LOGE("MEM", "Memory allocation failed");
        esp_camera_fb_return(pic);
        return false;
    }

    memcpy(output_img.data, pic->buf, pic->len);

    esp_camera_fb_return(pic);
    return true;
}

// Capture task: captures 50-200 images at CAPTURE_INTERVAL_MS, stops when weight drops.
// Then sends all images via MQTT. Signals first_image_sem after capturing the first image.
static void capture_and_send_task(void *pvParameters) {
    s_session.image_count = 0;

    // Flush stale frame buffers from the camera driver.
    // With fb_count=2 and CAMERA_GRAB_WHEN_EMPTY, both buffers contain
    // old frames from before this session started.
    for (int i = 0; i < camera_config.fb_count; i++) {
        camera_fb_t *stale = esp_camera_fb_get();
        if (stale) {
            esp_camera_fb_return(stale);
        }
    }

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(CAPTURE_INTERVAL_MS);

    for (int i = 0; i < MAX_CAPTURE_IMAGES; i++) {
        vTaskDelayUntil(&last_wake, interval);

        // After minimum images, check if weight is still present
        if (i >= MIN_CAPTURE_IMAGES) {
            float w = hx711_read_grams();
            if (w < HX711_WEIGHT_THRESHOLD_G) {
                ESP_LOGI("CAPTURE", "Weight gone after %d images, stopping capture", i);
                break;
            }
        }

        dl::image::jpeg_img_t img;
        if (!capture_image(img)) {
            ESP_LOGW("CAPTURE", "Failed to capture image %d, skipping", i);
            continue;
        }

        // Store image (data is malloc'd by capture_image)
        s_session.images[s_session.image_count] = img;
        s_session.image_count++;

        // image_count updated; classify_task waits for CAPTURE_DONE_BIT
    }

    ESP_LOGI("CAPTURE", "Captured %d images. Waiting for weight confirmation...", s_session.image_count);

    // Wait for the main loop to confirm or reject the weight reading
    EventBits_t bits = xEventGroupWaitBits(s_session.done_event,
                                           WEIGHT_CONFIRMED_BIT | WEIGHT_REJECTED_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WEIGHT_REJECTED_BIT) {
        ESP_LOGW("CAPTURE", "Weight not confirmed — discarding %d images", s_session.image_count);
        xEventGroupSetBits(s_session.done_event, CAPTURE_DONE_BIT);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("CAPTURE", "Weight confirmed. Sending %d images via MQTT...", s_session.image_count);

    for (int i = 0; i < s_session.image_count; i++) {
        if (s_session.images[i].data) {
            mqtt::send_image(s_session.images[i].data, s_session.images[i].data_size);
            vTaskDelay(pdMS_TO_TICKS(20)); // pace MQTT publishes
        }
    }

    ESP_LOGI("CAPTURE", "All images sent via MQTT");
    xEventGroupSetBits(s_session.done_event, CAPTURE_DONE_BIT);
    vTaskDelete(NULL);
}

// Classification task: classifies every 10th image as it becomes available during capture,
// then sends the majority class with its average confidence score via MQTT.
static void classify_task(void *pvParameters) {
    // Classify every 10th image and accumulate per-class stats
    std::map<std::string, std::pair<int, float>> class_stats; // class -> (vote_count, score_sum)
    int next_classify_idx = 0;

    // Process images as they arrive; stop when capture is done and no more remain
    while (true) {
        // If weight was rejected, bail out early
        EventBits_t wbits = xEventGroupGetBits(s_session.done_event);
        if (wbits & WEIGHT_REJECTED_BIT) {
            ESP_LOGW("CLASSIFY", "Weight not confirmed — skipping classification");
            xEventGroupSetBits(s_session.done_event, CLASSIFY_DONE_BIT);
            vTaskDelete(NULL);
            return;
        }

        bool capture_done = (wbits & CAPTURE_DONE_BIT) != 0;

        if (s_session.image_count > next_classify_idx) {
            dl::image::img_t decoded_img;
            decoded_img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

            esp_err_t decode_err = dl::image::sw_decode_jpeg(s_session.images[next_classify_idx], decoded_img, true);
            if (decode_err != ESP_OK) {
                ESP_LOGW("CLASSIFY", "Failed to decode JPEG at index %d, skipping", next_classify_idx);
            } else {
                dl::cls::result_t res = run_inference(s_model, s_preprocessor, decoded_img);
                if (res.cat_name) {
                    ESP_LOGI("CLASSIFY", "[img %d] %s (%.4f)", next_classify_idx, res.cat_name, res.score);
                    auto &entry = class_stats[res.cat_name];
                    entry.first++;
                    entry.second += res.score;
                }
                free(decoded_img.data);
                decoded_img.data = nullptr;
            }
            next_classify_idx += 10;
        } else if (capture_done) {
            break;
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    int count = s_session.image_count;
    ESP_LOGI("CLASSIFY", "Classification done (%d images classified). Total captured: %d", next_classify_idx / 10, count);

    if (class_stats.empty()) {
        ESP_LOGW("CLASSIFY", "No valid predictions obtained");
        xEventGroupSetBits(s_session.done_event, CLASSIFY_DONE_BIT);
        vTaskDelete(NULL);
        return;
    }

    // Find the majority class (most votes); use average confidence for that class
    std::string majority_class;
    int max_votes = 0;
    float majority_score_sum = 0.0f;
    for (auto &kv : class_stats) {
        if (kv.second.first > max_votes) {
            max_votes = kv.second.first;
            majority_score_sum = kv.second.second;
            majority_class = kv.first;
        }
    }
    float avg_confidence = majority_score_sum / max_votes;

    ESP_LOGI("CLASSIFY", "Result: %s avg_score=%.4f (votes=%d/%zu)",
             majority_class.c_str(), avg_confidence, max_votes, class_stats.size());

    dl::cls::result_t avg_result = {};
    avg_result.cat_name = majority_class.c_str();
    avg_result.score    = avg_confidence;

    mqtt::send_classification(avg_result, s_session.initial_weight_g);

    // Log entry to SD card
    time_t ts;
    time(&ts);
    struct tm tm_info;
    localtime_r(&ts, &tm_info);

    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &tm_info);

    char line[160];
    snprintf(line, sizeof(line), "%s %s avg_score=%.4f votes=%d",
             timestamp, majority_class.c_str(), avg_confidence, max_votes);

    if (!sdcard::append_line(background_log_path, line)) {
        ESP_LOGE("SD", "Failed to append log");
    }

    // Save the first image as a representative sample
    if (count > 0 && s_session.images[0].data) {
        if (!sdcard::save_classified_jpeg(s_session.images[0], avg_result, out_dir)) {
            ESP_LOGE("SD", "Failed to save classified JPEG");
        }
    }

    xEventGroupSetBits(s_session.done_event, CLASSIFY_DONE_BIT);
    vTaskDelete(NULL);
}


extern "C" void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(200));

    // Enable WiFi modem sleep to reduce power while polling
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

    obtain_time();

    // SD card
    ESP_LOGI(TAG, "Mounting SD card...");
    if (!sdcard::init()) {
        ESP_LOGE(TAG, "SD card init/mount failed");
        return;
    }
    sdcard::create_dir(out_dir);
    snprintf(background_log_path, sizeof(background_log_path), "%s/background_log.txt", out_dir);

    // Camera
    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed");
        return;
    }

    // MQTT
    ESP_LOGI(TAG, "Starting MQTT...");
    mqtt::mqtt_app_start();

    // Load classification model once (kept in PSRAM for the lifetime of the device)
    char dir[64];
    snprintf(dir, sizeof(dir), "%s/espdl_models", CONFIG_BSP_SD_MOUNT_POINT);
    s_model = new dl::Model(model_path, dir, static_cast<fbs::model_location_type_t>(0));
    if (!s_model) {
        ESP_LOGE(TAG, "Failed to create model");
        return;
    }
    s_model->minimize();

    s_preprocessor = new dl::image::ImagePreprocessor(s_model, {123.675, 116.28, 103.53}, {58.395, 57.12, 57.375}, DL_IMAGE_CAP_RGB565_BIG_ENDIAN);
    if (!s_preprocessor) {
        ESP_LOGE(TAG, "Failed to create image preprocessor");
        return;
    }

    // Scale init + tare
    hx711_init(HX711_DATA_PIN, HX711_CLK_PIN);
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Taring scale...");
    hx711_tare(20);

    time_t now; time(&now);
    ESP_LOGI("TIME", "epoch=%lld", (long long)now);
    ESP_LOGI(TAG, "Setup complete. Monitoring scale (threshold=%.1f g)...", HX711_WEIGHT_THRESHOLD_G);

    // Main loop: poll scale, launch capture+classify sessions on weight detection
    for (;;) {
        float weight = hx711_read_grams();

        if (weight >= HX711_WEIGHT_THRESHOLD_G) {
            ESP_LOGI(TAG, "Weight detected: %.2f g — starting session (pending confirmation)", weight);

            // Initialize session
            memset(&s_session, 0, sizeof(s_session));
            s_session.initial_weight_g = weight;
            s_session.done_event = xEventGroupCreate();

            // Launch capture task on core 0, classification task on core 1
            xTaskCreatePinnedToCore(capture_and_send_task, "capture",  8192 * 2, NULL, 19, NULL, 0);
            xTaskCreatePinnedToCore(classify_task,         "classify", 8192 * 2, NULL, 20, NULL, 1);

            // Wait 500 ms for HX711 to be ready, then take a confirmation reading
            vTaskDelay(pdMS_TO_TICKS(500));
            float weight2 = hx711_read_grams();
            if (weight2 >= HX711_WEIGHT_THRESHOLD_G) {
                float confirmed = (weight + weight2) / 2.0f;
                s_session.initial_weight_g = confirmed;
                ESP_LOGI(TAG, "Weight confirmed: %.2f g (%.2f + %.2f)", confirmed, weight, weight2);
                xEventGroupSetBits(s_session.done_event, WEIGHT_CONFIRMED_BIT);
            } else {
                ESP_LOGI(TAG, "False trigger (%.2f g then %.2f g) — aborting session", weight, weight2);
                xEventGroupSetBits(s_session.done_event, WEIGHT_REJECTED_BIT);
            }

            // Wait for both tasks to finish
            xEventGroupWaitBits(s_session.done_event,
                                CAPTURE_DONE_BIT | CLASSIFY_DONE_BIT,
                                pdFALSE, pdTRUE, portMAX_DELAY);

            // Free all captured image data
            for (int i = 0; i < s_session.image_count; i++) {
                if (s_session.images[i].data) {
                    free(s_session.images[i].data);
                    s_session.images[i].data = nullptr;
                }
            }

            vEventGroupDelete(s_session.done_event);

            ESP_LOGI(TAG, "Session complete (%d images). Waiting for weight to clear...", s_session.image_count);

            // Wait for weight to drop before next session
            while (hx711_read_grams() >= HX711_WEIGHT_THRESHOLD_G) {
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            ESP_LOGI(TAG, "Weight cleared. Ready for next detection.");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}