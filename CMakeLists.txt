cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR
        "In-source builds are not supported. Use:\n"
        "  cmake -S . -B build && cmake --build build")
endif()

project(dual_arm_v2_2_control_sdk VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install"
        CACHE PATH "Install path prefix" FORCE)
endif()
message(STATUS "Dual Arm V2.2 SDK install prefix: ${CMAKE_INSTALL_PREFIX}")

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

add_subdirectory(cpp)

option(DUAL_ARM_V2_2_SDK_BUILD_EXAMPLES "Build SDK example programs" ON)
if(DUAL_ARM_V2_2_SDK_BUILD_EXAMPLES)
    set(_dual_arm_example_dir ${CMAKE_SOURCE_DIR}/examples)

    function(add_sdk_example target)
        add_executable(${target} ${_dual_arm_example_dir}/${target}.cpp)
        target_include_directories(${target} PRIVATE ${_dual_arm_example_dir})
        target_link_libraries(${target} PRIVATE dual_arm_v2_2_control_sdk)
    endfunction()

    # Getting started
    add_sdk_example(go_home)
    add_sdk_example(move_arms_demo)

    # Applications
    add_sdk_example(teach_replay)

    # Validation / advanced
    add_sdk_example(test_control_api)
    add_sdk_example(test_control_api_various_command_frequency)
    add_sdk_example(test_move_joint_curve_fit)
    add_sdk_example(test_gripper_api)
    add_sdk_example(test_dual_gripper_api)
    add_sdk_example(test_arm_gripper_safe)
    add_sdk_example(test_dual_arm_100hz_follow)

    unset(_dual_arm_example_dir)
endif()
