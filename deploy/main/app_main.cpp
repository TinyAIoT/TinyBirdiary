#include "esp_log.h"
#include "dl_model_base.hpp"
#include "dl_image_define.hpp"
#include "dl_image_preprocessor.hpp"
#include "dl_cls_postprocessor.hpp"
#include "dl_image_jpeg.hpp"
#include "bsp/esp-bsp.h"
#include <esp_system.h>
#include <string.h>

#include "esp_jpeg_enc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "include/ClassificationPostProcessor.hpp"
#include "include/classification_category_name.hpp"
#include "esp_camera.h"

#include "sd_card.hpp"
#include "mqtt.hpp"
#include "include/bird_classification.hpp"
#include "include/image_ring_buffer.hpp"

#include <dirent.h>
#include <vector>
#include <limits>

#include "include/camera_pins.h"


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
#define WIFI_SSID      "..."
#define WIFI_PASSWORD      "..."
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
static const char *TAG = "wifi station";

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

extern const uint8_t example_jpg_start[] asm("_binary_example_jpg_start");
extern const uint8_t example_jpg_end[] asm("_binary_example_jpg_end");
extern const uint8_t espdl_model[] asm("_binary_birdiary_v5_squeezenet_layerwise_equalization_2_10_2_espdl_start");
static const char *model_path = (const char *)espdl_model;


static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static const int MAX_RETRY = 5;

ImageRingBuffer ring_buffer;
const char *out_dir = "/sdcard/classification";
char background_log_path[128];

bool pause_capture = false;

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

jpeg_error_t encode_img_to_jpeg(dl::image::img_t *img, dl::image::jpeg_img_t *jpeg_img, jpeg_enc_config_t jpeg_enc_cfg)
{
    jpeg_enc_handle_t jpeg_enc = NULL;
    jpeg_error_t ret = jpeg_enc_open(&jpeg_enc_cfg, &jpeg_enc);
    if (ret != JPEG_ERR_OK)
    {
        return ret;
    }

    int outbuf_size = 100 * 1024; // 100 KB
    uint8_t *outbuf = (uint8_t *)calloc(1, outbuf_size);
    if (!outbuf)
    {
        jpeg_enc_close(jpeg_enc);
        return JPEG_ERR_NO_MEM;
    }

    int out_len = 0;
    ret = jpeg_enc_process(jpeg_enc, (const uint8_t *)img->data, img->width * img->height * 3, outbuf, outbuf_size, &out_len);
    if (ret == JPEG_ERR_OK)
    {
        jpeg_img->data = outbuf;
        jpeg_img->data_size = out_len;
        jpeg_img->width = img->width;
        jpeg_img->height = img->height;
    }
    else
    {
        free(outbuf);
    }

    jpeg_enc_close(jpeg_enc);
    return ret;
}

static void camera_capture_task(void *pvParameters) {
    TickType_t cLastWakeTime = xTaskGetTickCount();
    const TickType_t cFrequency = pdMS_TO_TICKS(150); // 143 ms
    uint32_t last_capture_time = 0;
    // rolling buffer for camera capture FPS (kept outside loop so initialized once)
    static float capture_fps_buf[50] = {0};
    static int capture_fps_buf_idx = 0;
    static int capture_fps_buf_count = 0; // number of valid entries (<=50)
    for (;;) {
        // delay until maximum frequency (keep `cLastWakeTime` across iterations)
        vTaskDelayUntil(&cLastWakeTime, cFrequency);
        // calc and print framerate
        uint32_t now = xTaskGetTickCount();
        if (!pause_capture && last_capture_time != 0) {
            float seconds = (now - last_capture_time) * portTICK_PERIOD_MS / 1000.0f;
            if (seconds > 0.0f) {
                float fps = 1.0f / seconds;

                // push into circular buffer
                capture_fps_buf[capture_fps_buf_idx] = fps;
                capture_fps_buf_idx = (capture_fps_buf_idx + 1) % 50;
                if (capture_fps_buf_count < 50) capture_fps_buf_count++;

                float sum = 0.0f;
                for (int i = 0; i < capture_fps_buf_count; ++i) sum += capture_fps_buf[i];
                float avg = sum / (float)capture_fps_buf_count;

                ESP_LOGW("CAM", "Capture framerate: %.2f FPS (avg last %d: %.2f FPS)", fps, capture_fps_buf_count, avg);
            }
        }
        last_capture_time = now;
        if (pause_capture) {
            ESP_LOGI("CAM", "Capture paused, skipping this frame");
            continue;
        }

        ESP_LOGI("CAM", "Free heap at start of loop: %lu bytes", esp_get_free_heap_size());

        dl::image::jpeg_img_t img;
        if (!capture_image(img)) {
            ESP_LOGE("CAM", "Could not take picture");
            free(img.data);
            img.data = nullptr;
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        ESP_LOGI("RINGBUFFER", "free heap size before malloc: %zu", esp_get_free_heap_size());
        if (!ring_buffer.add_image(img)) {
            ESP_LOGE("RINGBUFFER", "Could not add image to ring buffer");

            free(img.data);
            img.data = nullptr;
            continue;
        }

        free(img.data);
        img.data = nullptr;
    }
}

static void classification_task(void *pvParameters) {
    ESP_LOGI("CLASSIFY", "start (core=%d, tick=%u)", xPortGetCoreID(), xTaskGetTickCount());
    uint32_t last_capture_time = 0;
    if (!initialize_bird_model()) {
        ESP_LOGE("CLASSIFY", "Failed to initialize bird model");
        vTaskDelete(NULL);
    }
    
    // Initialize model once at the start of classification task
    char dir[64];
    snprintf(dir, sizeof(dir), "%s/espdl_models", CONFIG_BSP_SD_MOUNT_POINT);
    dl::Model *model = new dl::Model(model_path, dir, static_cast<fbs::model_location_type_t>(0));
    if (!model) {
        ESP_LOGE("MODEL", "Failed to create model");
        vTaskDelete(NULL);
    }
    model->minimize();
    vTaskDelay(pdMS_TO_TICKS(10));

    dl::image::ImagePreprocessor *m_image_preprocessor = nullptr;

    m_image_preprocessor = new dl::image::ImagePreprocessor(model, {123.675, 116.28, 103.53}, {58.395, 57.12, 57.375}, DL_IMAGE_CAP_RGB565_BIG_ENDIAN);
    if (!m_image_preprocessor) {
        ESP_LOGE("PREPROCESSOR", "Failed to create image preprocessor");
        vTaskDelete(NULL);
    }
    
    // rolling buffer for classification FPS (kept outside loop so initialized once)
    static float classification_fps_buf[50] = {0};
    static int classification_fps_buf_idx = 0;
    static int classification_fps_buf_count = 0; // number of valid entries (<=16)
    for (;;) {
        // calc and print framerate
        uint32_t now = xTaskGetTickCount();
        if (last_capture_time != 0) {
            float seconds = (now - last_capture_time) * portTICK_PERIOD_MS / 1000.0f;
            if (seconds > 0.0f) {
                float fps = 1.0f / seconds;
                classification_fps_buf[classification_fps_buf_idx] = fps;
                classification_fps_buf_idx = (classification_fps_buf_idx + 1) % 50;
                if (classification_fps_buf_count < 50) classification_fps_buf_count++;

                float sum = 0.0f;
                for (int i = 0; i < classification_fps_buf_count; ++i) sum += classification_fps_buf[i];
                float avg = sum / (float)classification_fps_buf_count;

                ESP_LOGW("CLASSIFY", "Classification framerate: %.2f FPS (avg last %d: %.2f FPS)", fps, classification_fps_buf_count, avg);
            }
        }
        last_capture_time = now;

        dl::image::jpeg_img_t *img_jpeg = ring_buffer.get_latest_image();
        if (!img_jpeg->data) {
            ESP_LOGE("CLASSIFY", "No image in ring buffer");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // dl::image::jpeg_img_t img_jpeg;
        // if (!capture_image(img_jpeg)) {
        //     ESP_LOGE("CAM", "Could not take picture");
        //     free(img_jpeg.data);
        //     img_jpeg.data = nullptr;
        //     vTaskDelay(pdMS_TO_TICKS(2000));
        //     continue;
        // }
        
        // Decode JPEG BEFORE freeing encoded data
        dl::image::img_t decoded_img;
        decoded_img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
        esp_err_t decode_err = ESP_OK;
        
        if (img_jpeg->data) {
            decode_err = dl::image::sw_decode_jpeg(*img_jpeg, decoded_img, true);

            if (decode_err != ESP_OK) {
                ESP_LOGE("TESTI", "Failed to decode JPEG");
            } else {
                // Run inference
                dl::cls::result_t best = run_inference(model, m_image_preprocessor, decoded_img);
                if (best.cat_name) {
                    ESP_LOGI("INF", "Prediction: %s (score: %.4f)", best.cat_name, best.score);
                } else {
                    ESP_LOGW("INF", "No valid prediction");
                }
                
                // Save to SD and upload via WiFi if not background class
                if (best.cat_name != classification_cat_names[0]) {
                    pause_capture = true;
                    // Save to SD card
                    if (!sdcard::save_classified_jpeg(*img_jpeg, best, out_dir)) {
                        ESP_LOGE("SD", "Failed to save classified JPEG");
                    }
                
                    // MQTT
                    mqtt::send_classification(best);
                    int index = ring_buffer.get_latest_index() + 1;
                    for (int i = 0; i < RING_BUFFER_SIZE; ++i) {
                        dl::image::jpeg_img_t *img_to_send = ring_buffer.get_image((index + RING_BUFFER_SIZE) % RING_BUFFER_SIZE);
                        ESP_LOGI("MQTT", "Sending image index %d", (index + RING_BUFFER_SIZE) % RING_BUFFER_SIZE);  
                        if (img_to_send && img_to_send->data) {
                            mqtt::send_image(img_to_send->data, img_to_send->data_size);
                        }
                        index++;
                    }
                    pause_capture = false;
                } else {
                    // Append a log entry whenever background is detected
                    time_t ts;
                    time(&ts);
                    struct tm tm_info;
                    localtime_r(&ts, &tm_info);

                    char timestamp[32];
                    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &tm_info);

                    char line[128];
                    const char *cat = best.cat_name ? best.cat_name : "background";
                    snprintf(line, sizeof(line), "%s %s score=%.4f", timestamp, cat, best.score);

                    if (!sdcard::append_line(background_log_path, line)) {
                        ESP_LOGE("SD", "Failed to append background log");
                    } else {
                        ESP_LOGI("SD", "Background classification logged");
                    }
                }
            }

            // Free decoded image data
            if (decoded_img.data) {
                free(decoded_img.data);
                decoded_img.data = nullptr;
            }
        } else {
            ESP_LOGE("TESTI", "Skipping decode/inference due to previous JPEG encode error");
        }


        // Now free the encoded data
        if (img_jpeg->data) {
            free(img_jpeg->data);
            img_jpeg->data = nullptr;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (model) {
        delete model;
        model = nullptr;
    }
    if (m_image_preprocessor) {
        delete m_image_preprocessor;
        m_image_preprocessor = nullptr;
    }
}

// In app_main(), create a time sync task
static void time_sync_task(void *pvParameters) {
    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(1200000)); // Every 20 minutes
        obtain_time();
        ESP_LOGI("TIME", "Time re-synchronized");
    }
}


extern "C" void app_main(void) {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Initialize WiFi
    ESP_LOGI("WIFI", "Initializing WiFi...");
    wifi_init_sta();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    obtain_time();

    ESP_LOGI("SD", "Mounting SD card...");
    bool mounted = sdcard::init();
    if (!mounted) {
        ESP_LOGE("SD", "SD card init/mount failed");
        return;
    }

    if (ESP_OK != init_camera()) {
        ESP_LOGE("APP", "Camera initialization failed");
        return;
    }
    
    sdcard::create_dir(out_dir);

    snprintf(background_log_path, sizeof(background_log_path), "%s/background_log.txt", out_dir);

    time_t now; time(&now);
    ESP_LOGI("TIME", "epoch=%lld", (long long)now);

    // connect to mqtt
    ESP_LOGI("MQTT", "start...");
    mqtt::mqtt_app_start();

    xTaskCreatePinnedToCore(camera_capture_task,            "camera",           8192*2, NULL, 19, NULL, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(classification_task,            "classification",   8192*2, NULL, 20, NULL, 1);
    xTaskCreate(time_sync_task, "time_sync", 4096, NULL, 5, NULL);
}