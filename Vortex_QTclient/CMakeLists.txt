cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0144 NEW)

set(Qt6_ROOT "C:/Qt/6.10.2/mingw_64")
set(GST_ROOT "C:/Users/23585/Desktop/CODE/Tools/mingw_x86_64")
list(APPEND CMAKE_PREFIX_PATH ${Qt6_ROOT} ${GST_ROOT})

project(Vortex_QTclient LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Network)
qt_standard_project_setup()

qt_add_executable(Vortex_QTclient
    main.cpp
    mainwindow.h mainwindow.cpp
    videoplayer.h videoplayer.cpp
    cmdclient.h cmdclient.cpp
)

target_include_directories(Vortex_QTclient PRIVATE
    ${GST_ROOT}/include
    ${GST_ROOT}/include/gstreamer-1.0
    ${GST_ROOT}/include/glib-2.0
    ${GST_ROOT}/lib/glib-2.0/include
)

find_library(GLIB_LIB glib-2.0 PATHS ${GST_ROOT}/lib NO_DEFAULT_PATH)
find_library(GOBJECT_LIB gobject-2.0 PATHS ${GST_ROOT}/lib NO_DEFAULT_PATH)
find_library(GSTREAMER_LIB gstreamer-1.0 PATHS ${GST_ROOT}/lib NO_DEFAULT_PATH)
find_library(GSTBASE_LIB gstbase-1.0 PATHS ${GST_ROOT}/lib NO_DEFAULT_PATH)
find_library(GSTVIDEO_LIB gstvideo-1.0 PATHS ${GST_ROOT}/lib NO_DEFAULT_PATH)
find_library(GSTAPP_LIB gstapp-1.0 PATHS ${GST_ROOT}/lib NO_DEFAULT_PATH)

if(NOT GLIB_LIB OR NOT GOBJECT_LIB OR NOT GSTREAMER_LIB OR NOT GSTBASE_LIB OR NOT GSTVIDEO_LIB OR NOT GSTAPP_LIB)
    message(FATAL_ERROR "GStreamer/GLib 依赖库缺失，请检查GST_ROOT路径")
endif()

target_link_libraries(Vortex_QTclient PRIVATE
    Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Network
    ${GLIB_LIB} ${GOBJECT_LIB}
    ${GSTREAMER_LIB} ${GSTBASE_LIB}
    ${GSTVIDEO_LIB} ${GSTAPP_LIB}
)

# ==== 新增：调试阶段自动复制 GStreamer DLL ====
file(GLOB GST_DLLS "${GST_ROOT}/bin/*.dll")
add_custom_command(TARGET Vortex_QTclient POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${GST_DLLS}
    $<TARGET_FILE_DIR:Vortex_QTclient>
    COMMENT "复制 GStreamer DLL 到输出目录"
)

# ==== 新增：复制 GStreamer 插件目录 ====
add_custom_command(TARGET Vortex_QTclient POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${GST_ROOT}/lib/gstreamer-1.0
    $<TARGET_FILE_DIR:Vortex_QTclient>/lib/gstreamer-1.0
    COMMENT "复制 GStreamer 插件到输出目录"
)

# ==== 新增：运行时提示 ====
message(STATUS "========================================")
message(STATUS "调试运行前请设置环境变量：")
message(STATUS "  set GST_PLUGIN_PATH=${GST_ROOT}/lib/gstreamer-1.0")
message(STATUS "========================================")

# Qt 官方部署脚本（仅 cmake --install 时生效）
qt_generate_deploy_app_script(
    TARGET Vortex_QTclient
    OUTPUT_SCRIPT deploy_script
)
install(SCRIPT ${deploy_script})

install(TARGETS Vortex_QTclient
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)