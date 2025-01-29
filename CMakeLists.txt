cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
    ${extra_components_dirs_append}
)

project(yogyui-esp32-rs485-example)
idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17;-Os" APPEND)
idf_build_set_property(CXX_COMPILE_OPTIONS "-Os" APPEND)
idf_build_set_property(CXX_COMPILE_OPTIONS "-DCHIP_HAVE_CONFIG_H" APPEND)
idf_build_set_property(CXX_COMPILE_OPTIONS "-fpermissive" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-Os" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-unused-function" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-format-nonliteral" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-format-security" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wformat=0" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=maybe-uninitialized" APPEND)
