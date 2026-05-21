cmake_minimum_required(VERSION 3.10)

project(sentinel_ros
        VERSION 0.1
        LANGUAGES CXX)

# ---------------- C++ 标准 ----------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------- 编译选项 ----------------
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-Wall -Wextra -Wpedantic -O0 -g)
else()
    add_compile_options(-O3 -DNDEBUG)
endif()

# ---------------- ROS2 必须 ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

# ---------------- libusb ----------------
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
include_directories(${LIBUSB_INCLUDE_DIRS})
link_directories(${LIBUSB_LIBRARY_DIRS})

# ---------------- 头文件 ----------------
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

# ---------------- 源文件 ----------------
set(SRC_FILES
    src/sentinel_node.cpp
    src/DecodePacket.cpp
    src/Device.cpp
    src/FrameBuffer.cpp
)

# ---------------- 可执行文件 ----------------
add_executable(sentinel_node ${SRC_FILES})

ament_target_dependencies(sentinel_node
    rclcpp
    sensor_msgs
    cv_bridge
    image_transport
)

target_link_libraries(sentinel_node
    ${LIBUSB_LIBRARIES}
    ${OpenCV_LIBS}
    pthread
)

# ---------------- 编译选项 ----------------
target_compile_options(sentinel_node PRIVATE
    -O2
    -Wall
    -Wno-unused-variable
)

# ---------------- 安装 ----------------
install(TARGETS sentinel_node
    DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()