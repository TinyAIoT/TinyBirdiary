#include "bird_classification.hpp"

extern const uint8_t espdl_bird_model[] asm("_binary_birdiary_v5_squeezenet_layerwise_equalization_2_10_2_espdl_start");
dl::Model *bird_model = nullptr;
dl::image::ImagePreprocessor *m_bird_preprocessor = nullptr;

bool initialize_bird_model() {    
    bird_model = new dl::Model((const char *)espdl_bird_model);
    if (!bird_model) {
        ESP_LOGE("bird", "Failed to create model");
        return false;
    }

    m_bird_preprocessor = new dl::image::ImagePreprocessor(bird_model, {123.675, 116.28, 103.53}, {58.395, 57.12, 57.375});
    if (!m_bird_preprocessor) {
        ESP_LOGE("bird", "Failed to create image preprocessor");
        delete bird_model;
        bird_model = nullptr;
        return false;
    }

    return true;
}


const std::vector<dl::cls::result_t> run_bird_inference(dl::image::img_t &input_img) {   
    std::vector<dl::cls::result_t> results;
    // TODO 
    // uint32_t t0, t1;
    // float delta;
    // t0 = esp_timer_get_time();
    
    // m_bird_preprocessor->preprocess(input_img);

    // bird_model->run(dl::RUNTIME_MODE_MULTI_CORE);
    // const int check = 5;
    // birdPostProcessor m_postprocessor(bird_model, check, std::numeric_limits<float>::lowest(), true);
    // std::vector<dl::cls::result_t> &results = m_postprocessor.postprocess();

    // t1 = esp_timer_get_time();
    // delta = t1 - t0;
    // printf("Inference in %8.0f us.\n", delta);

    // for (auto &res : results) {
    //     ESP_LOGI("bird", "category: %s, score: %f\n", res.cat_name, res.score);
    // }

    return results;
}
