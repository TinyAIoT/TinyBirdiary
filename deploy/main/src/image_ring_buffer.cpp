#include "image_ring_buffer.hpp"

ImageRingBuffer::ImageRingBuffer() {
    for (int i = 0; i < RING_BUFFER_SIZE; ++i) {
        images_[i].data     = nullptr;
        images_[i].height   = 0;
        images_[i].width    = 0;
    }
    write_index_ = 0;
    count_ = 0;
}

ImageRingBuffer::~ImageRingBuffer() {
    for (int i = 0; i < RING_BUFFER_SIZE; ++i) {
        if (images_[i].data) {
            free(images_[i].data);
            images_[i].data = nullptr;
        }
    }
}

bool ImageRingBuffer::add_image(const dl::image::jpeg_img_t& img) {
    size_t data_size = img.data_size;
    
    ESP_LOGI("RINGBUFFER", "write_index: %d, data_size: %zu", write_index_, data_size);
    
    // Free old buffer if size changed
    if (images_[write_index_].data) {
        size_t old_size = images_[write_index_].data_size;
        if (old_size != data_size) {
            free(images_[write_index_].data);
            images_[write_index_].data = nullptr;
        }
    }
    
    // Allocate if needed
    if (!images_[write_index_].data) {
        images_[write_index_].data = static_cast<uint8_t*>(malloc(data_size));
        if (!images_[write_index_].data) {
            ESP_LOGE("RINGBUFFER", "Memory allocation failed for ring buffer (size: %zu)", data_size);
            return false;
        }
        images_[write_index_].data_size = data_size;
    }
    
    // Convert RGB565 → RGB888 if needed
    // if (img.pix_type == dl::image::DL_IMAGE_PIX_TYPE_RGB565) {
    //     dl::image::img_t temp_rgb888;
    //     temp_rgb888.height = img.height;
    //     temp_rgb888.width = img.width;
    //     temp_rgb888.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
    //     temp_rgb888.data = images_[write_index_].data;
        
    //     dl::image::convert_img(img, temp_rgb888, 0, nullptr, {});
    // } else {
        // Already RGB888, just copy
        memcpy(images_[write_index_].data, img.data, data_size);
    // }
    
    images_[write_index_].height = img.height;
    images_[write_index_].width = img.width;
    
    write_index_ = (write_index_ + 1) % RING_BUFFER_SIZE;
    if (count_ < RING_BUFFER_SIZE) {
        count_++;
    }
    
    return true;
}

bool ImageRingBuffer::is_full() { 
    return count_ == RING_BUFFER_SIZE; 
}

int ImageRingBuffer::get_count() { 
    return count_; 
}

int ImageRingBuffer::get_latest_index() {
    return (write_index_ - 1 + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
}

dl::image::jpeg_img_t* ImageRingBuffer::get_latest_image() {
    int latest_index = get_latest_index();
    ESP_LOGW("RINGBUFFER", "get_latest_image called, write_index_: %d", latest_index);
    return get_image(latest_index);
}

dl::image::jpeg_img_t* ImageRingBuffer::get_image(int index) {
    if (index >= count_) {
        return nullptr;
    }
    int actual_index = (write_index_ - count_ + index + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
    return &images_[actual_index];
}