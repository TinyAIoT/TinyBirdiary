#include <vector>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include "dl_image_define.hpp"
#include "esp_camera.h"
#include "esp_log.h"
#include "dl_image_preprocessor.hpp"


#define RING_BUFFER_SIZE 24

class ImageRingBuffer {
public:
    ImageRingBuffer();
    ~ImageRingBuffer();

    bool add_image(const dl::image::jpeg_img_t& img);

    bool is_full();

    int get_count();

    int get_latest_index();

    dl::image::jpeg_img_t* get_latest_image();
    dl::image::jpeg_img_t* get_image(int index);

private:
    std::vector<std::vector<uint8_t>> buffer_;
    dl::image::jpeg_img_t images_[RING_BUFFER_SIZE];
    int write_index_;
    int count_;
    mutable std::mutex mutex_;
};