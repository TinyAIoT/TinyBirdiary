#include "dl_model_base.hpp"
#include "dl_image_define.hpp"
#include "dl_image_preprocessor.hpp"
#include "dl_cls_postprocessor.hpp"
#include "dl_image_jpeg.hpp"
#include "esp_log.h"
#include "esp_camera.h"

#include "ClassificationPostProcessor.hpp"
#include "classification_category_name.hpp"

bool initialize_bird_model();
std::vector<dl::cls::result_t> run_bird_inference(const dl::image::img_t &img);
