cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(omniperception OBJECT 
    decider.cpp
    perceptron.cpp
    # 【改动】全向相机的 PnP 胶水层（复用 auto_aim 的公共 PnP 内核）
    omni_pnp.cpp
)
target_link_libraries(omniperception openvino::runtime)