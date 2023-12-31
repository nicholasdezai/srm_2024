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
    add_compile_options(-O3 -march=x86-64)
  elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
    add_compile_options(-O3 -march=native)
  endif()
endif()

# 添加可执行文件
add_executable(${EXECUTABLE_NAME} main.cpp)

# 添加模块头文件
include_directories(
  modules/common/include
  modules/video/include
  modules/coord/include
  modules/nn/include
  modules/robot/include
  modules/core/include
)

# 编译模块 注意顺序问题
add_subdirectory(modules/common)
add_subdirectory(modules/video)
add_subdirectory(modules/coord)
add_subdirectory(modules/nn)
add_subdirectory(modules/robot)
add_subdirectory(modules/core)

# 链接库主程序
target_link_libraries(
  ${EXECUTABLE_NAME} 
  srm_core
)
