#pragma once

#include "dl_image_define.hpp"
#include "dl_cls_postprocessor.hpp"  // for dl::cls::result_t

namespace sdcard {

bool init();

bool create_dir(const char *full_path);

int count_files(const char *full_path);

bool save_classified_jpeg(dl::image::jpeg_img_t &img, const dl::cls::result_t &best, const char *dir_full_path);

// Append a single line to a text file (creates the file if missing).
bool append_line(const char *filepath, const char *line);

} // namespace sdcard
