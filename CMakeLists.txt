# 项目总配置
cmake_minimum_required(VERSION 3.12)
project(SRM-Vision-2024)
set(EXECUTABLE_NAME srm-vision-2024)

# 设置编译选项
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_IGNORE_PATH /opt/homebrew)
  set(CMAKE_PREFIX_PATH /usr/local)
  set(CMAKE_OSX_ARCHITECTURES x86_64)
  set(CMAKE_CXX_COMPILER clang++)
  # set(CMAKE_EXE_LINKER_FLAGS
      # "-framework AppKit -framework CoreML -framework Foundation -framework Vision -framework CoreVideo"
  # )
  # add_compile_options(-x objective-c++ -fobjc-arc)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_COMPILER g++) # Ubuntu 20.04 or 22.04
  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_COMPILER g++-8) # Ubuntu 18.04
  endif()
endif()

# 设置优化参数
if(CMAKE_BUILD_TYPE STREQUAL Debug)
  add_compile_options(-O0)
elseif(CMAKE_BUILD_TYPE STREQUAL Release)
  if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    add_compile_options(-O3 -march=x86-64 -flto)
  elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
    add_compile_options(-O3 -march=native -flto)
  endif()
endif()

# 设置目录位置
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(CUDA_PATH /usr/local/cuda) # Linux: CUDA 目录
  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(TensorRT_PATH /opt/TensorRT) # Linux x86-64: TensorRT 目录
  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(TensorRT_PATH /usr/src/tensorrt)
    set(TensorRT_LIB_PATH /usr/lib/aarch64-linux-gnu)
  endif()
endif()

# 查找通用软件包
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Glog REQUIRED)
find_package(gflags REQUIRED)


if(CMAKE_SYSTEM_NAME MATCHES "Darwin")

elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  find_package(CUDA REQUIRED)
  find_package(TensorRT REQUIRED)
endif()

# 添加通用头文件
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${gflags_INCLUDE_DIR})

# 添加模块头文件
include_directories(
  modules/common/include
  modules/video/include
  modules/coord/include
)

# 编译模块
add_subdirectory(modules/common)
add_subdirectory(modules/video)
add_subdirectory(modules/coord)

# 创建和链接主程序
add_executable(${EXECUTABLE_NAME} main.cpp)
target_link_libraries(
  ${EXECUTABLE_NAME}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${GLOG_LIBRARIES}
  gflags
)