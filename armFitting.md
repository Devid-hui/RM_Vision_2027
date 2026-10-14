# ARM 架构适配文档（armFitting.md）

> 分支：`pure_cpp_Serial_port_Arm`
> 目标：将项目从 x86_64 平台迁移到 ARM（aarch64）平台编译并运行
> 分析结论：代码层几乎没有架构相关硬伤，主要工作量在**工具链安装、OpenVINO 版本、推理设备配置、运行时库路径**四个方面。

---

## 0. 重要决策：OpenVINO 内置到 `three_part/`

**OpenVINO 不通过系统安装（不使用 `/opt/intel`），直接放入项目根目录 `three_part/`**，作为随项目分发的第三方预编译依赖。理由：

- x86 / ARM 各自在 `three_part/` 中放置对应架构的 OpenVINO，源码与 CMake 完全一致，**无需架构分支**；
- 换机器、重装系统无需重新安装配置 OpenVINO，放置目录即可编译；
- 版本锁定在项目内，避免系统升级导致行为漂移。

目标结构：

```
RM_Vision_2027/
├── three_part/                          # 第三方预编译依赖（架构相关二进制）
│   └── openvino_2024.6.0/               # OpenVINO 2024.6.0（aarch64 版）
│       └── runtime/
│           ├── cmake/                   # find_package(OpenVINO) 入口
│           ├── include/                 # C++ 头文件
│           └── lib/aarch64/             # 运行库 + plugins/（CPU 插件所在）
├── io/
│   ├── hikrobot/lib/arm64/libMvCameraControl.so    # 海康 SDK（已内置）
│   └── mindvision/lib/arm64/libMVSDK.so            # 迈德威视 SDK（已内置）
└── ...
```

> 注意：`three_part/` 体积约 1GB+，建议 `.gitignore` 排除或使用 git-lfs / 内网分发，不要直接提交二进制进 git。

---

## 1. 现状分析

### 1.1 分支状态

`git diff pure_cpp_Serial_port pure_cpp_Serial_port_Arm` 为空，即当前分支与上一分支内容**完全一致**，尚未开始任何 ARM 适配修改。

### 1.2 项目在 x86 上的运行方式（基线）

- 平台：x86_64 + Ubuntu 20.04/22.04
- 编译器：g++（C++17）
- 构建：CMake ≥ 3.16，`cmake -B build && make -C build -j$(nproc)`
- 推理：OpenVINO 2024.6.0（原为 x86 系统安装，现改为 `three_part/` 内置），部分配置使用 `device: GPU`（Intel 核显）

---

## 2. 依赖总览与 ARM 适配性分析

### 2.1 系统 / apt 依赖（均需在 ARM 上重新安装）

| 依赖 | 用途 | ARM(aarch64) 适配性 |
|---|---|---|
| `libopencv-dev` | 图像处理、`cv::dnn` 分类器 | 无问题，apt 原生 arm64 包 |
| `libeigen3-dev` | 线性代数（纯模板库） | 无问题，头文件库 |
| `libfmt-dev` | 日志格式化 | 无问题 |
| `libspdlog-dev` | 日志 | 无问题 |
| `libyaml-cpp-dev` | 配置文件解析 | 无问题 |
| `nlohmann-json3-dev` | JSON | 无问题，纯头文件 |
| `libusb-1.0-0-dev` | MindVision 相机 USB reset | 无问题，apt 原生 arm64 包 |
| `can-utils` | SocketCAN 命令行工具 | 无问题（`linux/can.h` 为内核 API，ARM Linux 同样支持） |
| `cmake / g++ / git / screen / openssh-server` | 工具链 | 无问题 |

### 2.2 工业相机 SDK（仓库内置预编译 .so）

| SDK | 库文件 | amd64 | arm64 | 动态依赖（arm64 实测 readelf） |
|---|---|---|---|---|
| 海康机器人 HikRobot MVS | `io/hikrobot/lib/*/libMvCameraControl.so` | 已有 | **已有** | `libpthread / libdl / libstdc++ / libm / libgcc_s / libc` |
| 迈德威视 MindVision | `io/mindvision/lib/*/libMVSDK.so` | 已有 | **已有** | `librt / libpthread / libdl / libm / libc` |

**结论：两个相机 SDK 的 arm64 库均已随仓库提供**，只依赖 glibc 基础库，无需额外安装运行库。这是本项目适配工作量小的重要原因。

仍需注意：

- 运行时必须让动态链接器找到这两个 `.so`（见 §5.4）。
- 相机为 USB 设备，ARM 工控机/开发板上需要配置 **udev 规则**（否则无权限访问 `/dev/bus/usb/*`），且 USB 控制器建议使用原生 USB3 端口。

### 2.3 OpenVINO 2024.6.0（**适配重点，已决策：内置 `three_part/`**）

| 项目 | x86 现状 | ARM 要求 |
|---|---|---|
| 获取方式 | 原为系统安装 `/opt/intel/openvino_2024.6.0/`（x86_64 版） | 下载 **aarch64 版**离线包，解压到 `three_part/openvino_2024.6.0/` |
| 可用推理设备 | CPU / GPU(iGPU) | **仅 CPU**（ARM 上无 Intel GPU/NPU 插件） |
| CMake 查找路径 | 硬编码 `/opt/intel/openvino_2024.6.0/runtime/cmake`（4 处，见 §3） | 改为 `${CMAKE_SOURCE_DIR}/three_part/openvino_2024.6.0/runtime/cmake`（见 §5.1） |

关键限制：

- **OpenVINO 的安装包是架构绑定的**，x86 上解压好的 OpenVINO 目录**不能**直接拷贝到 ARM 使用，必须单独下载 aarch64 版本；
- `runtime/lib/aarch64/` 下的 `plugins/` 子目录（CPU 插件）必须随库一起分发，不能只拷 `libopenvino.so`；
- 内置后 4 处 CMakeLists 的硬编码路径必须同步修改，否则 `find_package(OpenVINO)` 直接失败。

### 2.4 模型文件（架构无关，可直接复用）

| 文件 | 格式 | ARM 可用性 |
|---|---|---|
| `assets/yolo11.xml + yolo11.bin`、`assets/yolov5.*`、`assets/yolov8.*`、`assets/yolo11_buff_int8.*` | OpenVINO IR | 可用。IR 是中间表示，跨架构通用，ARM 上由 CPU 插件即时编译 |
| `assets/tiny_resnet.onnx`、`assets/best2-sim.onnx` | ONNX | 可用。`cv::dnn::readNetFromONNX` 与 OpenVINO 均可直接读取 |

> 提示：`yolo11_buff_int8` 是 INT8 量化 IR，量化标定在 x86 CPU 上完成，在 ARM CPU 上精度通常可接受，但建议实测确认。

### 2.5 ROS2（可选，仅哨兵多线程目标需要）

以下目标在检测到 ROS2 时才编译，其余全部目标不依赖 ROS2：

- `src/sentry_multithread.cpp`、`src/sentry_multithread_debug.cpp`、`tests/publish_test.cpp`
- `io` 库中的 `ros2/publish2nav.cpp`、`ros2/ros2.cpp`、`ros2/subscribe2nav.cpp`

ARM 上若使用 Ubuntu 22.04 arm64，可直接安装官方 ROS2 Humble arm64 包；若不需要哨兵链路，可完全不装 ROS2。

### 2.6 Ceres Solver（名存实亡的依赖）

- `tasks/auto_buff/CMakeLists.txt` 中 `find_package(Ceres REQUIRED)`，根 `CMakeLists.txt` 的 `auto_buff_debug*` 链接 `${CERES_LIBRARIES}`。
- 但全项目源码中**没有任何 `#include <ceres/...>`**（已全局搜索确认）。
- 处置建议二选一：
  1. 安装 `libceres-dev`（Ubuntu 22.04 arm64 为 Ceres 2.0.0）保持构建不变；
  2. 从 CMakeLists 中移除 Ceres 相关内容（推荐，可减少一个大型依赖）。

### 2.7 仓库内置源码库（无外部依赖，无适配问题）

- `io/serial/`：wjwwood/serial 的 vendored 副本（`rt/pthread`）。
- `tasks/auto_aim/planner/tinympc/`：vendored TinyMPC 静态库。
- 均为纯 C++ 源码编译，与架构无关。

---

## 3. CMakeLists 逐文件分析

### 3.1 根 `CMakeLists.txt`

| 行号 | 内容 | ARM 影响 |
|---|---|---|
| 11-16 | `find_package`: OpenCV / fmt / Eigen3 / spdlog / yaml-cpp / nlohmann_json | 均需 apt 安装 arm64 版本 |
| **19** | `set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")` | **硬编码系统安装路径**。需改为指向 `three_part/` 的相对路径（见 §5.1） |
| 20 | `find_package(OpenVINO REQUIRED)` | 依赖上一条路径 |
| 44-50 | 可执行文件链接 `${CERES_LIBRARIES}` | 根作用域中该变量为空（`find_package(Ceres)` 只在子目录），当前依赖 OBJECT 库传递链接，能工作但脆弱；与架构无关 |
| 109-131 | ROS2 条件编译 | 无 ROS2 时自动跳过，ARM 上行为一致 |

### 3.2 `io/CMakeLists.txt`

| 行号 | 内容 | ARM 影响 |
|---|---|---|
| **28-34** | `CMAKE_SYSTEM_PROCESSOR` 匹配 `aarch64` → 链接 `hikrobot/lib/arm64` | **已适配** ✅ |
| **38-44** | 同上，mindvision → `mindvision/lib/arm64` | **已适配** ✅ |
| 46 | `target_link_libraries(io MvCameraControl MVSDK usb-1.0 yaml-cpp serial)` | 需 `libusb-1.0-0-dev`；两个 SDK 库在对应 arm64 目录中存在 |
| 48-79 | ROS2 条件编译 | 可选 |

### 3.3 `tasks/auto_aim/CMakeLists.txt`、`tasks/auto_buff/CMakeLists.txt`、`tasks/omniperception/CMakeLists.txt`

- 三处同样硬编码 `set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")`（分别位于第 3/2/3 行），**需一并改为 `three_part/` 相对路径**（见 §5.1）。
- `auto_buff` 额外 `find_package(Ceres REQUIRED)`（见 §2.6）。
- 其余内容与架构无关。

### 3.4 `io/serial/CMakeLists.txt`、`tools/CMakeLists.txt`、`tasks/auto_aim/planner/tinympc/CMakeLists.txt`

无架构相关内容，无需改动。

---

## 4. 代码层分析

- **无 x86 SIMD intrinsics**：全局搜索 `immintrin.h / _mm_* / __x86_64__ / cpuid` 等仅命中海康 SDK 头文件 `MvISPErrorDefine.h` 内的错误码注释，无实际使用。
- **无端序相关代码**：未发现手写 `htole/le32` 等字节序处理，串口协议均按小端主机序直接 memcpy，ARM/x86 均为小端，行为一致。
- **SocketCAN**（`io/socketcan.hpp`）：使用内核 `linux/can.h` + epoll，内核 API 跨架构一致。
- **相机 SDK 调用**：海康仅枚举 `MV_USB_DEVICE`（USB 相机），MindVision 用 libusb reset，均无架构差异。
- **线程 / 时间戳**：`std::thread`、`std::chrono::steady_clock` 自建时间戳体系（见 README 约定 4），与平台无关。

---

## 5. 需要改动清单

### 5.1 【必须】OpenVINO 内置 `three_part/` + 修改 4 处 CMakeLists 路径

**步骤 1：放置 OpenVINO aarch64 版**

```bash
mkdir -p three_part && cd three_part
tar -xf openvino_2024.6.0_<aarch64 包名>.tgz   # 得到 three_part/openvino_2024.6.0/runtime/{cmake,include,lib/aarch64}
cd ..
```

**步骤 2：修改以下 4 处硬编码路径**（根 `CMakeLists.txt` 第 19 行；`tasks/auto_aim/CMakeLists.txt` 第 3 行；`tasks/auto_buff/CMakeLists.txt` 第 2 行；`tasks/omniperception/CMakeLists.txt` 第 3 行），统一替换为：

```cmake
if(NOT DEFINED OpenVINO_DIR)
    set(OpenVINO_DIR "${CMAKE_SOURCE_DIR}/three_part/openvino_2024.6.0/runtime/cmake")
endif()
```

要点：

- 用 `CMAKE_SOURCE_DIR`（始终指向顶层源码目录）而非 `PROJECT_SOURCE_DIR`，保证子目录 CMakeLists 中同样有效；
- 保留 `if(NOT DEFINED ...)` 包裹，便于临时用 `-DOpenVINO_DIR=...` 覆盖（例如临时切回系统安装版）；
- x86 分支同理：只需把 x86 版 OpenVINO 放到同一 `three_part/` 路径，**两端代码完全一致**。

**步骤 3：版本管理**

- `three_part/` 约 1GB+，建议加入 `.gitignore`，用 git-lfs 或内网分发/软链方式同步；
- 若团队约定直接入库，请确认 git 仓库空间与 clone 时间可接受。

### 5.2 【必须】修改配置文件中的推理设备

OpenVINO 在 ARM 上只有 CPU 插件，`device: GPU` 会直接抛异常。以下配置需将 `device` 改为 `"CPU"`：

| 文件 | 当前值 |
|---|---|
| `configs/ascento.yaml` | `GPU` |
| `configs/standard3.yaml` | `GPU` |
| `configs/sentry.yaml` | `GPU` |
| `configs/standard4.yaml` | `GPU` |
| `configs/mvs.yaml` | `GPU` |

（`example.yaml`、`demo.yaml`、`mtstandard.yaml`、`uav.yaml`、`sentry_example.yaml` 已是 `CPU`，无需改。）

### 5.3 【必须】README 补充 ARM 依赖安装指令

见 README「3.2 编译方式 → 安装依赖项」（已更新）：apt 清单不变；OpenVINO 改为下载 aarch64 包放置到 `three_part/`；相机 SDK 说明 arm64 库已内置、只需 udev 规则与运行时库路径。

### 5.4 【必须】运行时动态库路径

相机 SDK 与 OpenVINO 的运行库均在项目内（编译期靠 `target_link_directories` 链接），但**运行期**动态链接器默认不知道这些路径。推荐 ldconfig 一次性配置：

```bash
echo "$(pwd)/three_part/openvino_2024.6.0/runtime/lib/aarch64" | sudo tee /etc/ld.so.conf.d/openvino.conf
echo "$(pwd)/io/hikrobot/lib/arm64" | sudo tee /etc/ld.so.conf.d/mvs.conf
echo "$(pwd)/io/mindvision/lib/arm64" | sudo tee /etc/ld.so.conf.d/mvsdk.conf
sudo ldconfig
```

注意：

- OpenVINO 的 `runtime/lib/aarch64` 下的 `plugins/`（CPU 插件）必须整目录可被找到，只链 `libopenvino.so` 会在推理时找不到插件；
- 也可每次运行前 `export LD_LIBRARY_PATH=...`（三种方式详见 README 1.3）；
- 相机 SDK 的 `.so` 路径含 `$PWD`，ldconfig 配置写在绝对路径里，换部署目录后需重新生成 conf。

### 5.5 【必须】相机 udev 规则（USB 设备权限）

ARM 开发板/工控机同样需要，否则相机打开失败（`MV_E_ACCESS_DENIED` 类错误）：

- 海康：安装 MVS 自带的 udev 规则（如 `51-mvs.rules`，内容为海康相机 VID/PID 的 `MODE="0666"`）；
- 迈德威视：安装其 SDK 的 `*.rules`；
- 无桌面环境时这一点很容易被忽略，务必确认。

### 5.6 【建议】Ceres 依赖处置

见 §2.6。建议从 `tasks/auto_buff/CMakeLists.txt` 删除 `find_package(Ceres REQUIRED)` 与 `${CERES_LIBRARIES}` 链接，并删除根 `CMakeLists.txt` 第 49-50 行中的 `${CERES_LIBRARIES}`。若暂不改代码，则 ARM 上 `sudo apt install libceres-dev` 亦可。

### 5.7 【建议】部署脚本硬编码路径

- `autostart.sh` 中写死 `~/Desktop/sp_vision_25/` 与不存在的 `watchdog.sh`（未随仓库分发）。ARM 部署时需按实际路径修改，或将 `watchdog.sh` 纳入仓库。

### 5.8 【验证】推理精度与性能

- INT8 量化 IR（`yolo11_buff_int8`）在 ARM CPU 上的精度需实测；
- ARM CPU 无 AVX-512/AVX2 指令集，OpenVINO 会退化为 ARM 优化内核（fp16/neon），延迟与 x86 不同，需重新评估帧率预算；
- 若 SoC 带 NPU（RK3588 / Jetson Orin 等），OpenVINO 无对应插件，需另行评估（换 TensorRT / RKNN），属于超出本清单的额外工作。

---

## 6. 目标平台假设

- OS：Ubuntu 22.04 LTS (Jammy) **arm64 / aarch64**（全新系统，无预装）
- 推理后端：项目内置 `three_part/openvino_2024.6.0`（aarch64 版），`device: CPU`
- 相机：HikRobot USB 工业相机（主）/ MindVision（备）
- 不需要 ROS2 时可跳过全部 ROS 目标（CMake 自动跳过）

---

## 7. ARM 编译部署流程（汇总）

```bash
# 1) 系统依赖（完整清单见 README）
sudo apt install -y git g++ cmake can-utils libopencv-dev libfmt-dev libeigen3-dev \
    libspdlog-dev libyaml-cpp-dev libusb-1.0-0-dev nlohmann-json3-dev libceres-dev \
    openssh-server screen

# 2) 放置 OpenVINO（aarch64 版）到 three_part/
mkdir -p three_part && cd three_part
tar -xf openvino_2024.6.0_<aarch64 包名>.tgz   # 得到 three_part/openvino_2024.6.0/
cd ..

# 3) 运行时库路径（OpenVINO + 两个相机 SDK）
echo "$(pwd)/three_part/openvino_2024.6.0/runtime/lib/aarch64" | sudo tee /etc/ld.so.conf.d/openvino.conf
echo "$(pwd)/io/hikrobot/lib/arm64" | sudo tee /etc/ld.so.conf.d/mvs.conf
echo "$(pwd)/io/mindvision/lib/arm64" | sudo tee /etc/ld.so.conf.d/mvsdk.conf
sudo ldconfig

# 4) udev 规则（海康/迈德威视各自的 .rules 拷入 /etc/udev/rules.d/）
sudo cp <SDK中的>.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger

# 5) 配置文件 device: GPU -> CPU（见 §5.2）

# 6) 编译（4 处 CMakeLists 已按 §5.1 改为 three_part 相对路径）
cmake -B build && make -C build -j$(nproc)

# 7) 运行
./build/standard configs/standard3.yaml
```

---

## 8. 风险与检查清单

- [ ] `three_part/openvino_2024.6.0` 放置的是 **aarch64 版**（`runtime/lib/aarch64/` 存在），而非拷贝 x86 目录
- [ ] 4 处 CMakeLists 的 `OpenVINO_DIR` 已改为 `${CMAKE_SOURCE_DIR}/three_part/openvino_2024.6.0/runtime/cmake`
- [ ] 所有配置文件 `device` 均为 `CPU`（无 `GPU`）
- [ ] 运行时库可被链接器找到（`ldconfig -p | grep -E 'MvCameraControl|MVSDK|openvino'`），OpenVINO `plugins/` 目录在同一路径下
- [ ] 相机 udev 规则生效，非 root 可访问 USB 设备
- [ ] INT8 模型在 ARM CPU 上精度实测通过
- [ ] 帧率/延迟在 ARM CPU 上满足决策链路预算
- [ ] 若用哨兵链路：ROS2 Humble arm64 环境可用
- [ ] `three_part/` 的 git 管理方式已确认（.gitignore / LFS / 内网分发）
