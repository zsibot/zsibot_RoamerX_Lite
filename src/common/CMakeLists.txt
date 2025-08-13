cmake_minimum_required(VERSION 3.22)
project(common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(glog REQUIRED)
find_package(robots_dog_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/
                    ${rosbag2_cpp_INCLUDE_DIRS}
                    ${glog_SOURCE_DIR}/src  # 包含 glog 头文件路径
)


# 添加源文件
file(GLOB_RECURSE srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)

# 创建库目标
add_library(${PROJECT_NAME} SHARED ${srcs})

# 链接依赖库
target_link_libraries(${PROJECT_NAME} glog)

ament_target_dependencies(
  ${PROJECT_NAME}
  rcl
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  rosbag2_cpp
  robots_dog_msgs
)

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_export_libraries(${PROJECT_NAME})

# 安装头文件
install(DIRECTORY include/
        DESTINATION include/
)

install(TARGETS 
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
