cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0144 NEW)

# 路径改为可配置：优先读环境变量，否则用缓存变量（首次 configure 可用 -D 覆盖）
#   Windows: set Qt6_ROOT=C:/Qt/6.10.2/mingw_64
#            set GST_ROOT=C:/Tools/mingw_x86_64
if(DEFINED ENV{Qt6_ROOT})
    set(Qt6_ROOT "$ENV{Qt6_ROOT}")
else()
    set(Qt6_ROOT "C:/Qt/6.10.2/mingw_64" CACHE PATH "Qt6 安装根目录（含 mingw_64 子目录）")
endif()

if(DEFINED ENV{GST_ROOT})
    set(GST_ROOT "$ENV{GST_ROOT}")
else()
    set(GST_ROOT "C:/Tools/mingw_x86_64" CACHE PATH "GStreamer 安装根目录（含 bin/lib/include）")
endif()

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

# GUI 程序：不弹黑色控制台窗口（双击运行不再出现黑框）
set_target_properties(Vortex_QTclient PROPERTIES WIN32_EXECUTABLE TRUE)

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
list(FILTER GST_DLLS EXCLUDE REGEX ".*OpenCL\\.dll$")
add_custom_command(TARGET Vortex_QTclient POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${GST_DLLS}
    $<TARGET_FILE_DIR:Vortex_QTclient>
    COMMENT "复制 GStreamer DLL 到输出目录"
)

# ==== 复制 GStreamer 必要插件（白名单）====
# 注意：不能拷贝全部插件！MSYS2 的 GStreamer 插件目录里，
# fluidsynthmidi/wasapi/d3d11/kvsink 等设备类插件初始化时会枚举
# 音频/显卡设备，在无设备或驱动异常的系统上会挂死导致程序卡在黑屏。
# 本程序只需 UDP 视频流解码，白名单覆盖：udpsrc/rtpjitterbuffer/
# rtph264depay/h264parse/avdec_h264/videoconvert/videoscale/appsink。
set(GST_PLUGIN_DIR ${GST_ROOT}/lib/gstreamer-1.0)
set(GST_PLUGIN_WHITELIST
    ${GST_PLUGIN_DIR}/libgstapp.dll
    ${GST_PLUGIN_DIR}/libgstudp.dll
    ${GST_PLUGIN_DIR}/libgstrtpmanager.dll
    ${GST_PLUGIN_DIR}/libgstrtp.dll
    ${GST_PLUGIN_DIR}/libgstvideoparsersbad.dll
    ${GST_PLUGIN_DIR}/libgstlibav.dll
    ${GST_PLUGIN_DIR}/libgstvideoconvertscale.dll
    ${GST_PLUGIN_DIR}/libgstvideotestsrc.dll
    ${GST_PLUGIN_DIR}/libgstaudiotestsrc.dll
    ${GST_PLUGIN_DIR}/libgstcoreelements.dll
    ${GST_PLUGIN_DIR}/libgstcoretracers.dll
)
add_custom_command(TARGET Vortex_QTclient POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:Vortex_QTclient>/lib/gstreamer-1.0
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${GST_PLUGIN_WHITELIST}
    $<TARGET_FILE_DIR:Vortex_QTclient>/lib/gstreamer-1.0/
    COMMENT "复制 GStreamer 必要插件到输出目录"
)

# ==== 复制 Qt6 运行时 DLL（MSYS2 绿色版，qwindows 插件必需）====
set(QT_DLLS
    ${Qt6_ROOT}/bin/Qt6Core.dll
    ${Qt6_ROOT}/bin/Qt6Gui.dll
    ${Qt6_ROOT}/bin/Qt6Widgets.dll
    ${Qt6_ROOT}/bin/Qt6Network.dll
    ${Qt6_ROOT}/bin/Qt6OpenGL.dll
    ${Qt6_ROOT}/bin/Qt6OpenGLWidgets.dll
    ${Qt6_ROOT}/bin/Qt6Xml.dll
    ${Qt6_ROOT}/bin/Qt6DBus.dll
    ${Qt6_ROOT}/bin/Qt6Concurrent.dll
)
add_custom_command(TARGET Vortex_QTclient POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${QT_DLLS}
    $<TARGET_FILE_DIR:Vortex_QTclient>
    COMMENT "复制 Qt6 运行时 DLL 到输出目录"
)

# ==== 复制 Qt6 平台/样式/图片插件（缺 qwindows.dll 会启动即崩溃）====
add_custom_command(TARGET Vortex_QTclient POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:Vortex_QTclient>/platforms
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${Qt6_ROOT}/share/qt6/plugins/platforms/qwindows.dll
    $<TARGET_FILE_DIR:Vortex_QTclient>/platforms/
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:Vortex_QTclient>/styles
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${Qt6_ROOT}/share/qt6/plugins/styles/qmodernwindowsstyle.dll
    $<TARGET_FILE_DIR:Vortex_QTclient>/styles/
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:Vortex_QTclient>/imageformats
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${Qt6_ROOT}/share/qt6/plugins/imageformats/qgif.dll
    ${Qt6_ROOT}/share/qt6/plugins/imageformats/qico.dll
    ${Qt6_ROOT}/share/qt6/plugins/imageformats/qjpeg.dll
    $<TARGET_FILE_DIR:Vortex_QTclient>/imageformats/
    COMMENT "复制 Qt6 平台插件到输出目录"
)

# ==== 运行时提示 ====
message(STATUS "========================================")
message(STATUS "GStreamer 插件已复制到输出目录 lib/gstreamer-1.0，")
message(STATUS "程序启动时按 exe 相对路径自动加载，无需手动设置环境变量。")
message(STATUS "========================================")

# Qt 官方部署脚本（仅 cmake --install 时生效）
qt_generate_deploy_app_script(
    TARGET Vortex_QTclient
    OUTPUT_SCRIPT deploy_script
)
install(SCRIPT ${deploy_script})

# ==== 打包 GStreamer DLL 与必要插件（绿色版必需）====
install(FILES ${GST_DLLS} DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES ${GST_PLUGIN_WHITELIST} DESTINATION ${CMAKE_INSTALL_BINDIR}/lib/gstreamer-1.0)

install(TARGETS Vortex_QTclient
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)