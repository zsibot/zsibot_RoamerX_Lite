cmake_minimum_required(VERSION 3.22)

project(yaml-cpp)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
  add_compile_options(-w -fPIC)

endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../cmake)
include(GetYamlCpp)


install(TARGETS yaml-cpp
  EXPORT yaml-cpp
  LIBRARY DESTINATION lib
)


install(DIRECTORY ${yaml-cpp_SOURCE_DIR}/include/
   DESTINATION include/
)

install(EXPORT yaml-cpp
        FILE yaml-cpp-config.cmake
        DESTINATION lib/cmake/yaml-cpp)  # 安装导出的 CMake 配置文件

ament_export_targets(${PROJECT_NAME})