if(CONFIG_MODEL_MNASNET)
    message(FATAL_ERROR "MNASNET CURRENTLY NOT SUPPORTED")
endif()
if(CONFIG_MODEL_DENSENET)
    message(WARNING "DENSENET MIGHT BE TO BIG TO FIT INTO MEMORY")
endif()
set(src_dirs        ./
                    ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(include_dirs    ./
                    ${CMAKE_CURRENT_SOURCE_DIR}/include)

list(APPEND requires    esp32_s3_eye_noglib
                        esp_lcd
                        esp-dl
                        esp_wifi
                        nvs_flash
                        esp_event
                        esp_netif
                        esp_http_client
                        mqtt)

file(GLOB embed_imgs "${CMAKE_CURRENT_SOURCE_DIR}/data/*")
file(GLOB embed_models "${CMAKE_CURRENT_SOURCE_DIR}/models/*")
file(GLOB embed_cert "${CMAKE_CURRENT_SOURCE_DIR}/certs/*")

set(embed_files ${embed_imgs} ${embed_models} ${embed_cert})

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires} PRIV_REQUIRES nvs_flash fatfs EMBED_FILES ${embed_files})
