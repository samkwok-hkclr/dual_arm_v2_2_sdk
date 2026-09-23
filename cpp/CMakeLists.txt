cmake_minimum_required(VERSION 3.16)
project(dual_arm_v2_2_control_sdk LANGUAGES C CXX VERSION 0.1.0)

# ============================================================
# 0. 基础保护 & 标准
# ============================================================
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR
        "In-source builds are not supported.\n"
        "Use: cmake -S cpp -B cpp/build && cmake --build cpp/build")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 默认构建类型（仅单配置生成器）
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        Debug Release RelWithDebInfo MinSizeRel)
    message(STATUS "[SDK] CMAKE_BUILD_TYPE not set, defaulting to Release")
endif()

# ============================================================
# 1. 构建模式自动检测
#    - 可用 -DSDK_BUILD_AS_ROS2=ON/OFF 强制覆盖
# ============================================================
if(NOT DEFINED SDK_BUILD_AS_ROS2)
    if(DEFINED ENV{ROS_VERSION} OR DEFINED ENV{AMENT_PREFIX_PATH})
        set(SDK_BUILD_AS_ROS2 ON)
    else()
        set(SDK_BUILD_AS_ROS2 OFF)
    endif()
endif()

if(SDK_BUILD_AS_ROS2)
    message(STATUS "[SDK] Build mode = ROS2 (ament_cmake)")
    find_package(ament_cmake REQUIRED)
else()
    message(STATUS "[SDK] Build mode = plain CMake")
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)
endif()

# ============================================================
# 2. 依赖
# ============================================================
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
    set(YAML_CPP_LIB yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(YAML_CPP_LIB yaml-cpp)
else()
    set(YAML_CPP_LIB ${YAML_CPP_LIBRARIES})
endif()

# ============================================================
# 3. 路径
# ============================================================
get_filename_component(SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(ROBOT_PLATFORM_UTILS_DIR "${SDK_ROOT}/robot_platform_utils")
set(CURI_UDP_DIR             "${SDK_ROOT}/curi_udp")
set(SDK_CONFIG_YAML          "${SDK_ROOT}/cuarm_configuration/dual_v2_2/config.yaml")

# 安装后头文件放到 <prefix>/include/dual_arm_v2_2_sdk/
if(SDK_BUILD_AS_ROS2)
    set(SDK_INSTALL_INCLUDEDIR "include/dual_arm_v2_2_sdk")
else()
    set(SDK_INSTALL_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}/dual_arm_v2_2_sdk")
endif()

# ============================================================
# 4. 库 target —— 两种模式共用
# ============================================================
add_library(dual_arm_v2_2_control_sdk SHARED
    src/control_api.cpp
    ${ROBOT_PLATFORM_UTILS_DIR}/cpp/src/config_loader.cpp
    ${ROBOT_PLATFORM_UTILS_DIR}/cpp/src/cuarm_message_handler.cpp
    ${ROBOT_PLATFORM_UTILS_DIR}/cpp/src/time_utils.cpp
    ${CURI_UDP_DIR}/c/curi_udp.c
)
add_library(dual_arm_v2_2_control_sdk::dual_arm_v2_2_control_sdk
    ALIAS dual_arm_v2_2_control_sdk)

set_target_properties(dual_arm_v2_2_control_sdk PROPERTIES
    VERSION   ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    POSITION_INDEPENDENT_CODE ON
)

target_include_directories(dual_arm_v2_2_control_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${SDK_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${SDK_ROOT}
        ${ROBOT_PLATFORM_UTILS_DIR}/cpp/include
        ${CURI_UDP_DIR}/c
)

target_link_libraries(dual_arm_v2_2_control_sdk
    PUBLIC  Threads::Threads
    PRIVATE ${YAML_CPP_LIB}
)

# ============================================================
# 5. 编译选项 —— 两种模式共用（target-level，不污染下游）
# ============================================================
target_compile_options(dual_arm_v2_2_control_sdk PRIVATE
    # 警告（仅 GNU / Clang）
    $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall>
    $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wextra>
    $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wpedantic>

    # Debug
    $<$<CONFIG:Debug>:-O0;-g3;-fno-omit-frame-pointer>

    # Release —— 关键：显式补 -DNDEBUG
    $<$<CONFIG:Release>:-O2;-DNDEBUG>

    # RelWithDebInfo
    $<$<CONFIG:RelWithDebInfo>:-O2;-g;-DNDEBUG>

    # MinSizeRel
    $<$<CONFIG:MinSizeRel>:-Os;-DNDEBUG>
)

# ============================================================
# 6. 示例程序 —— 两种模式共用
# ============================================================
option(BUILD_SDK_EXAMPLES "Build SDK examples" OFF)

if(BUILD_SDK_EXAMPLES)
    # 顶层 examples/（cpp/ 的兄弟目录）
    set(EXAMPLE_INCLUDE_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}/../examples)

    function(add_sdk_example target)
        add_executable(${target}
            ${CMAKE_CURRENT_SOURCE_DIR}/../examples/${target}.cpp)
        target_include_directories(${target} PRIVATE ${EXAMPLE_INCLUDE_DIR})
        target_link_libraries(${target}
            PRIVATE dual_arm_v2_2_control_sdk::dual_arm_v2_2_control_sdk)
    endfunction()

    add_sdk_example(go_home)
    add_sdk_example(move_arms_demo)
    add_sdk_example(teach_replay)
    add_sdk_example(test_control_api)
    add_sdk_example(test_control_api_various_command_frequency)
    add_sdk_example(test_move_joint_curve_fit)
    add_sdk_example(test_gripper_api)
    add_sdk_example(test_dual_gripper_api)
    add_sdk_example(test_arm_gripper_safe)
endif()

# ============================================================
# 7. 安装 & 导出 —— 从这里开始分叉
# ============================================================

# ------------------------------------------------------------
# 7a. ROS2 模式
# ------------------------------------------------------------
if(SDK_BUILD_AS_ROS2)

    # ---- 库 ----
    install(TARGETS dual_arm_v2_2_control_sdk
        EXPORT  export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    # ---- 头文件 ----
    install(DIRECTORY include/
        DESTINATION ${SDK_INSTALL_INCLUDEDIR})

    # ---- 示例 ----
    if(BUILD_SDK_EXAMPLES)
        install(TARGETS
            go_home move_arms_demo teach_replay
            test_control_api test_control_api_various_command_frequency
            test_move_joint_curve_fit
            test_gripper_api test_dual_gripper_api test_arm_gripper_safe
            DESTINATION lib/${PROJECT_NAME})
    endif()

    # ---- config.yaml ----
    if(EXISTS "${SDK_CONFIG_YAML}")
        install(FILES ${SDK_CONFIG_YAML}
            DESTINATION share/${PROJECT_NAME}
            RENAME config.yaml)
    else()
        message(WARNING "[SDK] config.yaml not found at: ${SDK_CONFIG_YAML}")
    endif()

    # ---- ament 导出 ----
    ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
    ament_export_dependencies(Threads)
    # yaml-cpp 是 PRIVATE，不需要导出给下游；若公共头文件暴露了 YAML::Node 才加：
    # ament_export_dependencies(yaml-cpp)

    ament_package()

# ------------------------------------------------------------
# 7b. 裸 CMake 模式
# ------------------------------------------------------------
else()

    # ---- 库 ----
    install(TARGETS dual_arm_v2_2_control_sdk
        EXPORT  dual_arm_v2_2_control_sdkTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${SDK_INSTALL_INCLUDEDIR}
    )

    # ---- 头文件 ----
    install(DIRECTORY include/
        DESTINATION ${SDK_INSTALL_INCLUDEDIR})

    # ---- 示例 ----
    if(BUILD_SDK_EXAMPLES)
        install(TARGETS
            go_home move_arms_demo teach_replay
            test_control_api test_control_api_various_command_frequency
            test_move_joint_curve_fit
            test_gripper_api test_dual_gripper_api test_arm_gripper_safe
            DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()

    # ---- config.yaml ----
    if(EXISTS "${SDK_CONFIG_YAML}")
        install(FILES ${SDK_CONFIG_YAML}
            DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
            RENAME config.yaml)
    else()
        message(WARNING "[SDK] config.yaml not found at: ${SDK_CONFIG_YAML}")
    endif()

    # ---- CMake 包导出 ----
    set(SDK_CMAKE_INSTALL_DIR
        ${CMAKE_INSTALL_LIBDIR}/cmake/dual_arm_v2_2_control_sdk)

    install(EXPORT dual_arm_v2_2_control_sdkTargets
        FILE      dual_arm_v2_2_control_sdkTargets.cmake
        NAMESPACE dual_arm_v2_2_control_sdk::
        DESTINATION ${SDK_CMAKE_INSTALL_DIR})

    configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/dual_arm_v2_2_control_sdkConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/dual_arm_v2_2_control_sdkConfig.cmake
        INSTALL_DESTINATION ${SDK_CMAKE_INSTALL_DIR})

    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/dual_arm_v2_2_control_sdkConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion)

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/dual_arm_v2_2_control_sdkConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/dual_arm_v2_2_control_sdkConfigVersion.cmake
        DESTINATION ${SDK_CMAKE_INSTALL_DIR})
endif()