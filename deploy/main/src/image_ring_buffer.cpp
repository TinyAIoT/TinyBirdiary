#include "image_ring_buffer.hpp"

ImageRingBuffer::ImageRingBuffer() {
    for (int i = 0; i < RING_BUFFER_SIZE; ++i) {
        images_[i].data     = nullptr;
        images_[i].height   = 0;
        images_[i].width    = 0;
        images_[i].pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
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

bool ImageRingBuffer::add_image(const dl::image::img_t& img) {
    // Allocate new memory and copy image data
    size_t data_size = img.height * img.width * 3; // RGB888: 3 bytes per pixel
    
    if (images_[write_index_].data) {
        size_t old_size = images_[write_index_].height * images_[write_index_].width * 3;
        if (old_size != data_size) {
            free(images_[write_index_].data);
            images_[write_index_].data = nullptr;
        }
    }

    // Allocate if needed
    if (!images_[write_index_].data) {
        images_[write_index_].data = malloc(data_size);
        if (!images_[write_index_].data) {
            ESP_LOGE("TAKEOVER", "Memory allocation failed for ring buffer");
            return false;
        }
    }
    if (!images_[write_index_].data) {
        ESP_LOGE("TAKEOVER", "Memory allocation failed for ring buffer");
        return false;
    }
    
    memcpy(images_[write_index_].data, img.data, data_size);
    images_[write_index_].height = img.height;
    images_[write_index_].width = img.width;
    images_[write_index_].pix_type = img.pix_type;

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

dl::image::img_t* ImageRingBuffer::get_latest_image() {
    return get_image((write_index_ - 1) % RING_BUFFER_SIZE);
}

dl::image::img_t* ImageRingBuffer::get_image(int index) {
    if (index >= count_) {
        return nullptr;
    }
    int actual_index = (write_index_ - count_ + index + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
    return &images_[actual_index];
}