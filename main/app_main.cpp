#include "esp_log.h"
#include "dl_model_base.hpp"
#include "dl_image_define.hpp"
#include "dl_image_preprocessor.hpp"
#include "dl_cls_postprocessor.hpp"
#include "dl_image_jpeg.hpp"
#include "bsp/esp-bsp.h"
#include <esp_system.h>
#include <nvs_flash.h>
#include <string.h>
#include <sys/param.h>

#include "esp_jpeg_enc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/ClassificationPostProcessor.hpp"
#include "include/classification_category_name.hpp"
#include "esp_camera.h"
// SD , SPI
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "esp_camera.h"

#include <dirent.h>
#include <sys/stat.h>
#include <cstring>

// Support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

extern const uint8_t example_jpg_start[] asm("_binary_example_jpg_start");
extern const uint8_t example_jpg_end[] asm("_binary_example_jpg_end");
extern const uint8_t espdl_model[] asm("_binary_torch_mbnv2_layerwise_equalization_espdl_start");
static const char *model_path = (const char *)espdl_model;

// Set to true to take a camera picture, else make sure to add an img
#define TAKE_PICTURE true
#define SAVE_TO_SDCARD true

static const dl::cls::result_t run_inference(dl::image::img_t &input_img) {
    char dir[64];
    snprintf(dir, sizeof(dir), "%s/espdl_models", CONFIG_BSP_SD_MOUNT_POINT);

    dl::Model *model = nullptr;
    dl::image::ImagePreprocessor *m_image_preprocessor = nullptr;

    model = new dl::Model(model_path, dir, static_cast<fbs::model_location_type_t>(0));
    if (!model) {
        ESP_LOGE("MODEL", "Failed to create model");
        return {};
    }
    model->minimize();
    // ESP_ERROR_CHECK(model->test());

    // Add a small delay to ensure model is properly initialized
    vTaskDelay(pdMS_TO_TICKS(10));

    uint32_t t0, t1;
    float delta;
    t0 = esp_timer_get_time();
    m_image_preprocessor = new dl::image::ImagePreprocessor(model, {123.675, 116.28, 103.53}, {58.395, 57.12, 57.375}, DL_IMAGE_CAP_RGB565_BIG_ENDIAN);
    if (!m_image_preprocessor) {
        ESP_LOGE("PREPROCESSOR", "Failed to create image preprocessor");
        delete model;
        return {};
    }

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
            best_result = res; // Copy the result
            found_result = true;
        }
    }

    // Free resources
    if (m_image_preprocessor) {
        delete m_image_preprocessor;
        m_image_preprocessor = nullptr;
    }
    if (model) {
        delete model;
        model = nullptr;
    }

    return best_result;
}

#if SAVE_TO_SDCARD
#include "include/sd_pins.h"
#define MOUNT_POINT "/sdcard"
sdmmc_card_t *g_card;
int count_files_in_directory(const char *path) {
    int count = 0;
    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE("FILE_COUNT", "Failed to open directory: %s", path);
        return -1;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr) {
        // Skip current and parent directory entries
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        // Build full file path
        char full_path[256];
        // snprintf(full_path, sizeof(full_path), "%s/%s", path, entry->d_name);
        strlcpy(full_path, path, sizeof(full_path));
        strlcat(full_path, "/", sizeof(full_path));
        strlcat(full_path, entry->d_name, sizeof(full_path));
        // Get file info
        struct stat st;
        if (stat(full_path, &st) == 0) {
            if (S_ISREG(st.st_mode)) {
                count++;
            }
        }
        else {
            ESP_LOGW("FILE_COUNT", "Could not stat file: %s", full_path);
        }
    }

    closedir(dir);
    return count;
}

void createDir(const char *path) {
    static const char *TAG = "createDir";
    ESP_LOGI(TAG, "Creating Dir: %s", path);

    FRESULT res = f_mkdir(path);
    if (res == FR_OK) {
        ESP_LOGI(TAG, "Dir created");
    } else if (res == FR_EXIST) {
        ESP_LOGI(TAG, "Dir already exists");
    } else {
        ESP_LOGE(TAG, "mkdir failed with error: %d", res);
    }
}
void init_sd_enable_pin(void) {
    // Configure the GPIO as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SD_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    // Set the output level
    gpio_set_level(SD_ENABLE, 0);
}

bool mount_sdcard_spi() {
    init_sd_enable_pin();
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    // sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI("SD", "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI("SD", "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    // host.'slot' should be set to an sdspi device initialized by `sdspi_host_init_device()`.
    // SDSPI_HOST_DEFAULT: https://github.com/espressif/esp-idf/blob/1bbf04cb4cf54d74c1fe21ed12dbf91eb7fb1019/components/esp_driver_sdspi/include/driver/sdspi_host.h#L44
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 5000;
#define SPI_HOST_ID SPI3_HOST // #if SOC_SPI_PERIPH_NUM > 2 ???

    host.slot = SPI_HOST_ID; //

// For SoCs where the SD power can be supplied both via an internal or external (e.g. on-board LDO) power supply.
// When using specific IO pins (which can be used for ultra high-speed SDMMC) to connect to the SD card
// and the internal LDO power supply, we need to initialize the power supply first.
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_IO_ID,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
        return false;
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;
#endif
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ESP_LOGI("spi", "SDSPI_DEFAULT_DMA: %d", SDSPI_DEFAULT_DMA);
    ret = spi_bus_initialize(SPI_HOST_ID, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE("SD", "Failed to initialize bus (ret != ESP_OK).");

        return false;
    }
    // card select output ?
    gpio_reset_pin(PIN_NUM_CS);
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS, 1); // Inactive

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    // sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    sdspi_device_config_t slot_config = {
        .host_id = SPI_HOST_ID,
        .gpio_cs = PIN_NUM_CS,
        .gpio_cd = SD_SW,
        .gpio_wp = SDSPI_SLOT_NO_WP,
        .gpio_int = GPIO_NUM_NC,
        .gpio_wp_polarity = SDSPI_IO_ACTIVE_LOW,
        //.duty_cycle_pos = 0,
    };
    // spi_host_device_t host_id; ///< SPI host to use, SPIx_HOST (see spi_types.h)
    ESP_LOGI("SD", "Mounting filesystem");
    ESP_LOGI("MEM", "Free heap at A: %lu bytes", esp_get_free_heap_size());
    // gpio_set_level(SD_ENABLE, 1);
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &g_card);

    ESP_LOGI("MEM", "Free heap at B: %lu bytes", esp_get_free_heap_size());
    if (ret != ESP_OK) {
        ESP_LOGI("MEM", "Free heap at C: %lu bytes", esp_get_free_heap_size());
        if (ret == ESP_FAIL) {
            ESP_LOGE("SD", "Failed to mount filesystem (ret == ESP_FAIL). Look into esp_vfs_fat_sdspi_mount() "
                           "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE("SD", "Failed to initialize the card (%s). Look into esp_vfs_fat_sdspi_mount() "
                           "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
#ifdef CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
            check_sd_card_pins(&config, pin_count);
#endif
        }
        return false;
    }
    ESP_LOGI("SD", "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, g_card);
    return true;
};

jpeg_error_t encode_img_to_jpeg(dl::image::img_t *img, dl::image::jpeg_img_t *jpeg_img, jpeg_enc_config_t jpeg_enc_cfg) {
    jpeg_enc_handle_t jpeg_enc = NULL;
    jpeg_error_t ret = jpeg_enc_open(&jpeg_enc_cfg, &jpeg_enc);
    if (ret != JPEG_ERR_OK) {
        return ret;
    }

    int outbuf_size = 100 * 1024; // 100 KB
    uint8_t *outbuf = (uint8_t *)calloc(1, outbuf_size);
    if (!outbuf) {
        jpeg_enc_close(jpeg_enc);
        return JPEG_ERR_NO_MEM;
    }

    int out_len = 0;
    ret = jpeg_enc_process(jpeg_enc, (const uint8_t *)img->data, img->width * img->height * 3, outbuf, outbuf_size, &out_len);
    if (ret == JPEG_ERR_OK) {
        jpeg_img->data = outbuf;
        jpeg_img->data_size = out_len;
    } else {
        free(outbuf);
    }

    jpeg_enc_close(jpeg_enc);
    return ret;
}

// Returns a pointer to a static array of counts for each class
static std::vector<int> predict_sd_card_folder(const char *folder_path, const char *output_folder_path) {
    int num_classes = sizeof(classification_cat_names) / sizeof(classification_cat_names[0]);
    std::vector<int> class_counts(num_classes, 0);
    DIR *dir = opendir(folder_path);
    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr) {
        ESP_LOGI("HEAP", "Free heap at loop start: %lu bytes", esp_get_free_heap_size());
        // Skip current and parent directory entries
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        // Build full file path
        char *input_filepath = (char *)malloc(512);
        if (!input_filepath) {
            ESP_LOGE("SD", "Failed to allocate memory for filepath");
            continue;
        }
        int ret = snprintf(input_filepath, 512, "%s/%s", folder_path, entry->d_name);
        if (ret >= 512) {
            ESP_LOGE("SD", "Filepath too long for: %s", entry->d_name);
            free(input_filepath);
            continue;
        }

        // Check if it's a JPEG file
        const char *ext = strrchr(entry->d_name, '.');
        if (!ext || (strcasecmp(ext, ".jpg") != 0 && strcasecmp(ext, ".jpeg") != 0)) {
            ESP_LOGW("SD", "Skipping non-JPEG file: %s", entry->d_name);
            free(input_filepath);
            continue;
        }

        ESP_LOGI("SD", "Processing image: %s", entry->d_name);

        // Read JPEG file manually
        FILE *file = fopen(input_filepath, "rb");
        if (!file) {
            ESP_LOGE("SD", "Failed to open JPEG file: %s", entry->d_name);
            free(input_filepath);
            continue;
        }

        // We can free input_filepath now since file is opened
        free(input_filepath);

        // Get file size
        fseek(file, 0, SEEK_END);
        long file_size = ftell(file);
        fseek(file, 0, SEEK_SET);

        if (file_size <= 0) {
            ESP_LOGE("SD", "Invalid file size for: %s", entry->d_name);
            fclose(file);
            continue;
        }

        // Allocate memory and read file
        uint8_t *jpeg_data = (uint8_t *)malloc(file_size);
        if (!jpeg_data) {
            ESP_LOGE("SD", "Failed to allocate memory for JPEG data: %s", entry->d_name);
            fclose(file);
            continue;
        }

        size_t bytes_read = fread(jpeg_data, 1, file_size, file);
        fclose(file);

        if (bytes_read != file_size) {
            ESP_LOGE("SD", "Failed to read complete JPEG file: %s", entry->d_name);
            free(jpeg_data);
            continue;
        }

        // Create JPEG structure
        dl::image::jpeg_img_t jpeg_img;
        jpeg_img.data = jpeg_data;
        jpeg_img.width = 240;
        jpeg_img.height = 240;
        jpeg_img.data_size = file_size;

        // Convert JPEG to RGB888
        dl::image::img_t img;
        img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
        esp_err_t decode_err = dl::image::sw_decode_jpeg(jpeg_img, img, true);

        // Free JPEG data
        if (jpeg_img.data) {
            free(jpeg_img.data);
            jpeg_img.data = nullptr;
        }

        if (decode_err != ESP_OK) {
            ESP_LOGE("SD", "Failed to decode JPEG: %s", entry->d_name);
            continue;
        }

        // Run inference on image
        const auto best = run_inference(img);
        if (best.cat_name) {
            ESP_LOGI("CLS", "Best: %s (score: %f) for image: %s", best.cat_name, best.score, entry->d_name);
        }

        // Find predicted class index
        int pred_idx = -1;
        for (int i = 0; i < num_classes; ++i) {
            if (best.cat_name && strcmp(best.cat_name, classification_cat_names[i]) == 0) {
                pred_idx = i;
                break;
            }
        }
        if (pred_idx >= 0) {
            class_counts[pred_idx]++;
        }

        // Encode back to JPEG
        dl::image::jpeg_img_t encoded_jpeg_img;
        jpeg_enc_config_t enc_config = {
            .width = img.width,
            .height = img.height,
            .src_type = JPEG_PIXEL_FORMAT_RGB888,
            .subsampling = JPEG_SUBSAMPLE_444,
            .quality = 80,
            .rotate = JPEG_ROTATE_0D,
            .task_enable = true,
            .hfm_task_priority = 13,
            .hfm_task_core = 1,
        };

        jpeg_error_t encode_ret = encode_img_to_jpeg(&img, &encoded_jpeg_img, enc_config);

        // Free image data
        if (img.data) {
            free(img.data);
            img.data = nullptr;
        }

        if (encode_ret == JPEG_ERR_OK) {
            // Save classified image with prediction result
            char *output_filename = (char *)malloc(512);
            if (!output_filename) {
                ESP_LOGE("SD", "Failed to allocate memory for output filename");
                if (encoded_jpeg_img.data) {
                    heap_caps_free(encoded_jpeg_img.data);
                }
                continue;
            }

            const char *basename = entry->d_name;
            const char *dot = strrchr(basename, '.');
            ESP_LOGI("SD", "Basename: %s", basename);
            int name_ret;
            if (dot) {
                size_t name_len = dot - basename;
                char *name_without_ext = (char *)malloc(256);
                if (!name_without_ext) {
                    ESP_LOGE("SD", "Failed to allocate memory for name_without_ext");
                    free(output_filename);
                    if (encoded_jpeg_img.data) {
                        heap_caps_free(encoded_jpeg_img.data);
                    }
                    continue;
                }

                if (name_len >= 256) {
                    ESP_LOGE("SD", "Filename too long: %s", entry->d_name);
                    free(name_without_ext);
                    free(output_filename);
                    if (encoded_jpeg_img.data) {
                        heap_caps_free(encoded_jpeg_img.data);
                    }
                    continue;
                }
                strncpy(name_without_ext, basename, name_len);
                name_without_ext[name_len] = '\0';

                name_ret = snprintf(output_filename, 512,
                                    "classified/%s/%s_%s_%.4f.jpg",
                                    output_folder_path,
                                    name_without_ext,
                                    best.cat_name ? best.cat_name : "unknown",
                                    best.cat_name ? best.score : 0.0f);

                free(name_without_ext);
            } else {
                name_ret = snprintf(output_filename, 512,
                                    "classified/%s/%s_%s_%.2f.jpg",
                                    output_folder_path,
                                    basename,
                                    best.cat_name ? best.cat_name : "unknown",
                                    best.cat_name ? best.score : 0.0f);
            }

            if (name_ret >= 512) {
                ESP_LOGE("SD", "Output filename too long for: %s", entry->d_name);
                free(output_filename);
                if (encoded_jpeg_img.data) {
                    heap_caps_free(encoded_jpeg_img.data);
                }
                continue;
            }

            char output_filepath[512];
            int ret2 = snprintf(output_filepath, sizeof(output_filepath), "/sdcard/%s", output_filename);
            if (ret2 >= sizeof(output_filepath)) {
                ESP_LOGE("SD", "Output filepath too long for: %s", output_filename);
                free(output_filename);
                if (encoded_jpeg_img.data) {
                    heap_caps_free(encoded_jpeg_img.data);
                }
                continue;
            }

            esp_err_t write_err = dl::image::write_jpeg(encoded_jpeg_img, output_filepath);
            if (write_err == ESP_OK) {
                ESP_LOGI("SD", "Classified image saved: %s", output_filename);
            } else {
                ESP_LOGE("SD", "Failed to save classified image: %s", output_filepath);
            }

            // Free output_filename after we're done using it
            free(output_filename);

            // Free JPEG data
            if (encoded_jpeg_img.data) {
                heap_caps_free(encoded_jpeg_img.data);
            }
        } else {
            ESP_LOGE("JPEG", "Failed to encode image: %s", entry->d_name);
        }
    }

    closedir(dir);
    return class_counts;
}

#endif // SAVE_TO_SDCARD

#if TAKE_PICTURE
#include "include/camera_pins.h"

// Camera Module pin mapping
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

    .xclk_freq_hz = 20000000, // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_RGB565, // PIXFORMAT_RGB565 , PIXFORMAT_JPEG
    .frame_size = FRAMESIZE_QVGA,     // [<<320x240>> (QVGA, 4:3); FRAMESIZE_320X320, 240x176 (HQVGA, 15:11); 400x296 (CIF, 50:37)],FRAMESIZE_QVGA,FRAMESIZE_VGA

    .jpeg_quality = 8, // 0-63 lower number means higher quality.  Reduce quality if stack overflow in cam_task
    .fb_count = 2,     // if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = 0 // optional
};

static esp_err_t init_camera(void) {
    // Initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE("CAM", "Camera Init Failed");
    }
    return err;
}

static bool capture_image(dl::image::img_t &output_img) {
    ESP_LOGI("CAM", "Taking picture...");
    camera_fb_t *pic = esp_camera_fb_get();
    if (!pic) {
        ESP_LOGE("CAM", "Failed to capture image");
        return false;
    }

    // Use pic->buf to access the image
    ESP_LOGI("CAM", "Picture taken! Its size was: %zu bytes", pic->len);
    ESP_LOGW("image_dim", "Height: %d, Width: %d, Len: %zu", pic->height, pic->width, pic->len);

    // Allocate memory and copy the image data before returning the frame buffer
    output_img.height = pic->height;
    output_img.width = pic->width;
    output_img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565;

    // Allocate memory for the image data
    output_img.data = malloc(pic->len);
    if (!output_img.data) {
        ESP_LOGE("MEM", "Memory allocation failed");
        esp_camera_fb_return(pic);
        return false;
    }

    // Copy the image data
    memcpy(output_img.data, pic->buf, pic->len);

    esp_camera_fb_return(pic);
    return true;
}
#endif // TAKE_PICTURE

extern "C" void app_main(void) {
#if SAVE_TO_SDCARD
    bool mounted = false;
    ESP_LOGI("SD", "Mounting SD card..............");
    mounted = mount_sdcard_spi();
    vTaskDelay(pdMS_TO_TICKS(1000));
#endif

#if TAKE_PICTURE
    if (ESP_OK != init_camera()) {
        ESP_LOGE("SD", "Camera initialization failed. Look into init_camera()");
        return;
    }
    createDir("classification");

    while (mounted) {
        ESP_LOGI("MEM", "Free heap at start of loop: %lu bytes", esp_get_free_heap_size());
        dl::image::img_t img;
        if (!capture_image(img)) {
            ESP_LOGE("CAM", "Could not take picture");
            vTaskDelay(pdMS_TO_TICKS(2000));
            return;
        }

        // Convert RGB565 (camera) → RGB888 (needed for inference)
        dl::image::img_t rgb888_img;
        rgb888_img.height = img.height;
        rgb888_img.width = img.width;
        rgb888_img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
        rgb888_img.data = malloc(img.height * img.width * 3);
        if (!rgb888_img.data) {
            ESP_LOGE("MEM", "Failed to allocate RGB888 buffer");
            free(img.data);
            continue;
        }

        dl::image::convert_img(img, rgb888_img, 0, nullptr, {});

        // Free camera frame buffer
        free(img.data);

        // --- Run inference directly on captured image ---
        const auto best = run_inference(rgb888_img);
        if (best.cat_name) {
            ESP_LOGI("INF", "Prediction: %s (score: %.4f)", best.cat_name, best.score);
        } else {
            ESP_LOGW("INF", "No valid prediction");
        }

        // --- Encode and save the original image as JPEG ---
        dl::image::jpeg_img_t encoded_jpeg_img;
        jpeg_enc_config_t enc_config = {
            .width = rgb888_img.width,
            .height = rgb888_img.height,
            .src_type = JPEG_PIXEL_FORMAT_RGB888,
            .subsampling = JPEG_SUBSAMPLE_444,
            .quality = 80,
            .rotate = JPEG_ROTATE_0D,
            .task_enable = true,
            .hfm_task_priority = 13,
            .hfm_task_core = 1,
        };

        jpeg_error_t encode_ret = encode_img_to_jpeg(&rgb888_img, &encoded_jpeg_img, enc_config);

        // Free RGB888 data after encoding
        free(rgb888_img.data);

        if (encode_ret != JPEG_ERR_OK) {
            ESP_LOGE("JPEG", "JPEG encoding failed!");
            continue;
        }

        // Save JPEG with class label and score in filename
        int file_count = count_files_in_directory("/sdcard/classification");
        char filepath[128];
        snprintf(filepath, sizeof(filepath),
                "/sdcard/classification/%04d_%s_%.3f.jpg",
                file_count + 1,
                best.cat_name ? best.cat_name : "unknown",
                best.cat_name ? best.score : 0.0f);

        ESP_LOGI("SD", "Saving classified image: %s", filepath);
        esp_err_t write_err = dl::image::write_jpeg(encoded_jpeg_img, filepath);

        if (write_err == ESP_OK) {
            ESP_LOGI("SD", "Saved successfully");
        } else {
            ESP_LOGE("SD", "Failed to save JPEG");
        }

        heap_caps_free(encoded_jpeg_img.data);

        vTaskDelay(pdMS_TO_TICKS(2000)); // small delay before next capture
    }


#else // TAKE_PICTURE != true
#if SAVE_TO_SDCARD

    // check if there is a folder called "to classify" on the SD card
    const char *to_classify_path = "/sdcard/to_classify";

    if (opendir(to_classify_path)) {
        ESP_LOGI("SD", "Found to_classify folder, processing images...");

        createDir("classified");
        int num_classes = sizeof(classification_cat_names) / sizeof(classification_cat_names[0]);
        std::vector<std::vector<int>> confusion_matrix(num_classes, std::vector<int>(num_classes, 0));
        for (int true_idx = 0; true_idx < num_classes; ++true_idx) {
            const char *cat_name = classification_cat_names[true_idx];
            char out_path[128];
            snprintf(out_path, sizeof(out_path), "classified/%s", cat_name);
            createDir(out_path);

            char class_path[128];
            snprintf(class_path, sizeof(class_path), "/sdcard/to_classify/%s", cat_name);

            if (opendir(class_path)) {
                std::vector<int> predictions = predict_sd_card_folder(class_path, cat_name);
                ESP_LOGI("CLS", "Predictions for class %s:", cat_name);
                for (int pred_idx = 0; pred_idx < num_classes; ++pred_idx) {
                    ESP_LOGI("CLS", "  %s: %d", classification_cat_names[pred_idx], predictions[pred_idx]);
                    confusion_matrix[true_idx][pred_idx] = predictions[pred_idx];
                }
            } else {
                ESP_LOGE("SD", "Failed to open %s directory", class_path);
            }
        }

        ESP_LOGI("SD", "Finished processing all folders in to_classify folder");

        // Write confusion matrix to txt file
        FILE *cm_file = fopen("/sdcard/confusion_matrix.txt", "w");
        if (cm_file) {
            fprintf(cm_file, "Confusion Matrix\n\n");
            fprintf(cm_file, "                  Predicted\n");
            fprintf(cm_file, "                  ");
            for (int i = 0; i < num_classes; ++i)
                fprintf(cm_file, "%12s", classification_cat_names[i]);
            fprintf(cm_file, "\n");
            for (int i = 0; i < num_classes; ++i) {
                fprintf(cm_file, "Actual %13s", classification_cat_names[i]);
                for (int j = 0; j < num_classes; ++j) {
                    fprintf(cm_file, "%12d", confusion_matrix[i][j]);
                }
                fprintf(cm_file, "\n");
            }
            // overall accuracy
            int total = 0, correct = 0;
            for (int i = 0; i < num_classes; ++i) {
                for (int j = 0; j < num_classes; ++j) {
                    total += confusion_matrix[i][j];
                    if (i == j)
                        correct += confusion_matrix[i][j];
                }
            }
            float accuracy = (total > 0) ? ((float)correct / total) : 0.0f;
            fprintf(cm_file, "\nOverall Accuracy: %.2f%%\n", accuracy * 100.0f);
            fclose(cm_file);
            ESP_LOGI("SD", "Confusion matrix written to /sdcard/confusion_matrix.txt");
        } else {
            ESP_LOGE("SD", "Failed to write confusion matrix to /sdcard/confusion_matrix.txt");
        }
    } else {
        ESP_LOGI("SD", "No to_classify folder found, skipping batch processing");
    }

#else // SAVE_TO_SDCARD != true
    vTaskDelay(pdMS_TO_TICKS(1000));
    // example image is used
    dl::image::jpeg_img_t jpeg_img = {
        .data = (uint8_t *)example_jpg_start,
        .width = 240,
        .height = 240,
        .data_size = (uint32_t)(example_jpg_end - example_jpg_start),
    };

    dl::image::img_t img;
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
    esp_err_t decode_err = dl::image::sw_decode_jpeg(jpeg_img, img, true);

    if (decode_err != ESP_OK) {
        ESP_LOGE("SD", "Failed to decode JPEG");
        return;
    }

    const auto best = run_inference(img);
    if (best.cat_name) {
        ESP_LOGI("CLS", "Best: %s (score: %f)", best.cat_name, best.score);
    }
    heap_caps_free(img.data);
#endif // SAVE_TO_SDCARD
#endif // TAKE_PICTURE
}
