file(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.c)
idf_component_register(SRCS ${app_sources}
                       REQUIRES driver esp_rom freertos log)
