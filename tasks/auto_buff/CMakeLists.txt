cmake_minimum_required(VERSION 3.16)
# OpenVINO 路径解析（详见 armFitting.md §5.1）：
# - ARM(aarch64)：使用项目内置 3rd_part/openvino_2024.6.0（架构相关二进制，不进 git）
# - x86 手动解压安装：/opt/intel/openvino_2024.6.0
# - x86 apt 安装（libopenvino-dev）：不设置 OpenVINO_DIR，由 find_package 在系统路径搜索
# 任意场景均可通过 -DOpenVINO_DIR=<path> 覆盖
if(NOT DEFINED OpenVINO_DIR)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        set(OpenVINO_DIR "${CMAKE_SOURCE_DIR}/3rd_part/openvino_2024.6.0/runtime/cmake")
    elseif(EXISTS "/opt/intel/openvino_2024.6.0/runtime/cmake")
        set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
    endif()
endif()

find_package(OpenVINO REQUIRED)

add_library(auto_buff OBJECT 
    buff_type.cpp
    buff_detector.cpp
    buff_solver.cpp
    buff_aimer.cpp
    buff_target.cpp
    yolo11_buff.cpp
)

target_link_libraries(auto_buff auto_aim openvino::runtime)