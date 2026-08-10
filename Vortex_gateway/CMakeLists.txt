cmake_minimum_required(VERSION 3.10)
project(stream_app C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 使用pkg-config查找依赖
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(OPENCV REQUIRED opencv4)

# 源码列表
set(SRC_FILES
    main.cpp
    gst_task.cpp
    serial_task.cpp
    server.cpp
)

# 生成可执行程序
add_executable(stream_app ${SRC_FILES})

# 头文件路径
target_include_directories(stream_app PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${GSTREAMER_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(stream_app
    ${GSTREAMER_LIBRARIES}
    ${GST_APP_LIBRARIES}
    ${OPENCV_LIBRARIES}
    pthread
)

# 编译选项
target_compile_options(stream_app PRIVATE
    ${GSTREAMER_CFLAGS_OTHER}
    ${GST_APP_CFLAGS_OTHER}
    ${OPENCV_CFLAGS_OTHER}
)
