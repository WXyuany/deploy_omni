cmake_minimum_required(VERSION 3.16)

project(adapter_any_robot LANGUAGES CXX)

set(BRIDGE_ROBOT "" CACHE STRING
  "Robot adapter to build: kepler_k2, pm01, unitree_g1, agibot_x2, cvte_carm, or all")
set_property(CACHE BRIDGE_ROBOT PROPERTY STRINGS
  kepler_k2 pm01 unitree_g1 agibot_x2 cvte_carm all legacy)

# Keep the per-adapter switches for older build scripts. New builds should use
# BRIDGE_ROBOT so an SDK unavailable on the target machine is never selected.
option(BUILD_ROBOT_KEPLER_K2 "Build kepler_k2 adapter support" OFF)
option(BUILD_ROBOT_PM01 "Build pm01 adapter support" OFF)
option(BUILD_ROBOT_UNITREE_G1 "Build unitree_g1 adapter support" OFF)
option(BUILD_ROBOT_CVTE_CARM "Build CVTE CArm adapter support" OFF)
option(BUILD_ROBOT_AGIBOT_X2 "Build Agibot X2 adapter support" OFF)
option(BUILD_GAMEPAD_SDL "Build SDL2 gamepad input support" ON)
option(BUILD_UNITREE_G1_HARDWARE_TESTS "Build Unitree G1 hardware smoke tests" OFF)
option(BUILD_UNITREE_G1_MOTION_DEMO
  "Build the interactive Unitree G1 high-level motion demo" OFF)

if("${BRIDGE_ROBOT}" STREQUAL "")
  if(BUILD_ROBOT_KEPLER_K2 OR BUILD_ROBOT_PM01 OR BUILD_ROBOT_UNITREE_G1 OR
     BUILD_ROBOT_CVTE_CARM OR BUILD_ROBOT_AGIBOT_X2)
    set(BRIDGE_ROBOT "legacy")
    message(WARNING
      "BUILD_ROBOT_* switches are deprecated; use BRIDGE_ROBOT for new build configurations.")
  else()
    message(FATAL_ERROR
      "BRIDGE_ROBOT is required. Choose one of: kepler_k2, pm01, unitree_g1, agibot_x2, cvte_carm, all. "
      "Use BRIDGE_ROBOT=legacy only for the deprecated BUILD_ROBOT_* switches.")
  endif()
elseif("${BRIDGE_ROBOT}" STREQUAL "legacy")
  message(WARNING
    "BRIDGE_ROBOT=legacy is deprecated; use one explicit robot selection instead.")
elseif("${BRIDGE_ROBOT}" STREQUAL "all")
  set(BUILD_ROBOT_KEPLER_K2 ON)
  set(BUILD_ROBOT_PM01 ON)
  set(BUILD_ROBOT_UNITREE_G1 ON)
  set(BUILD_ROBOT_AGIBOT_X2 OFF)
  # CVTE has a separate binary SDK and is enabled explicitly to avoid making
  # the aggregate build depend on a machine-local vendor package.
  set(BUILD_ROBOT_CVTE_CARM OFF)
elseif("${BRIDGE_ROBOT}" STREQUAL "kepler_k2")
  set(BUILD_ROBOT_KEPLER_K2 ON)
  set(BUILD_ROBOT_PM01 OFF)
  set(BUILD_ROBOT_UNITREE_G1 OFF)
  set(BUILD_ROBOT_CVTE_CARM OFF)
  set(BUILD_ROBOT_AGIBOT_X2 OFF)
elseif("${BRIDGE_ROBOT}" STREQUAL "pm01")
  set(BUILD_ROBOT_KEPLER_K2 OFF)
  set(BUILD_ROBOT_PM01 ON)
  set(BUILD_ROBOT_UNITREE_G1 OFF)
  set(BUILD_ROBOT_CVTE_CARM OFF)
  set(BUILD_ROBOT_AGIBOT_X2 OFF)
elseif("${BRIDGE_ROBOT}" STREQUAL "unitree_g1")
  set(BUILD_ROBOT_KEPLER_K2 OFF)
  set(BUILD_ROBOT_PM01 OFF)
  set(BUILD_ROBOT_UNITREE_G1 ON)
  set(BUILD_ROBOT_CVTE_CARM OFF)
  set(BUILD_ROBOT_AGIBOT_X2 OFF)
elseif("${BRIDGE_ROBOT}" STREQUAL "agibot_x2" OR "${BRIDGE_ROBOT}" STREQUAL "x2")
  set(BUILD_ROBOT_KEPLER_K2 OFF)
  set(BUILD_ROBOT_PM01 OFF)
  set(BUILD_ROBOT_UNITREE_G1 OFF)
  set(BUILD_ROBOT_CVTE_CARM OFF)
  set(BUILD_ROBOT_AGIBOT_X2 ON)
elseif("${BRIDGE_ROBOT}" STREQUAL "cvte_carm")
  set(BUILD_ROBOT_KEPLER_K2 OFF)
  set(BUILD_ROBOT_PM01 OFF)
  set(BUILD_ROBOT_UNITREE_G1 OFF)
  set(BUILD_ROBOT_CVTE_CARM ON)
  set(BUILD_ROBOT_AGIBOT_X2 OFF)
else()
  message(FATAL_ERROR
    "Unsupported BRIDGE_ROBOT='${BRIDGE_ROBOT}'. Choose one of: "
    "kepler_k2, pm01, unitree_g1, agibot_x2, cvte_carm, all.")
endif()

if("${BRIDGE_ROBOT}" STREQUAL "cvte_carm" OR "${BRIDGE_ROBOT}" STREQUAL "agibot_x2" OR "${BRIDGE_ROBOT}" STREQUAL "x2")
  set(BUILD_BRIDGE_DDS_DEFAULT OFF)
else()
  set(BUILD_BRIDGE_DDS_DEFAULT ON)
endif()
option(BUILD_BRIDGE_DDS
  "Build the CycloneDDS runtime and DDS bridge components"
  ${BUILD_BRIDGE_DDS_DEFAULT})

if((BUILD_ROBOT_KEPLER_K2 OR BUILD_ROBOT_PM01 OR BUILD_ROBOT_UNITREE_G1) AND
   NOT BUILD_BRIDGE_DDS)
  message(FATAL_ERROR
    "The selected robot requires BUILD_BRIDGE_DDS=ON. Only cvte_carm supports an SDK-only build.")
endif()

if(BUILD_UNITREE_G1_MOTION_DEMO AND NOT BUILD_ROBOT_UNITREE_G1)
  message(FATAL_ERROR
    "BUILD_UNITREE_G1_MOTION_DEMO requires BRIDGE_ROBOT=unitree_g1")
endif()

if(NOT BUILD_ROBOT_KEPLER_K2 AND NOT BUILD_ROBOT_PM01 AND NOT BUILD_ROBOT_UNITREE_G1 AND NOT BUILD_ROBOT_CVTE_CARM AND NOT BUILD_ROBOT_AGIBOT_X2)
  message(FATAL_ERROR "No robot adapter selected. Set BRIDGE_ROBOT explicitly.")
endif()

set(BRIDGE_BUILD_KEPLER_K2 0)
if(BUILD_ROBOT_KEPLER_K2)
  set(BRIDGE_BUILD_KEPLER_K2 1)
endif()

set(BRIDGE_BUILD_PM01 0)
if(BUILD_ROBOT_PM01)
  set(BRIDGE_BUILD_PM01 1)
endif()

set(BRIDGE_BUILD_UNITREE_G1 0)
if(BUILD_ROBOT_UNITREE_G1)
  set(BRIDGE_BUILD_UNITREE_G1 1)
endif()

set(BRIDGE_BUILD_CVTE_CARM 0)
if(BUILD_ROBOT_CVTE_CARM)
  set(BRIDGE_BUILD_CVTE_CARM 1)
endif()

set(BRIDGE_BUILD_AGIBOT_X2 0)
if(BUILD_ROBOT_AGIBOT_X2)
  set(BRIDGE_BUILD_AGIBOT_X2 1)
endif()

set(BRIDGE_BUILD_DDS 0)
if(BUILD_BRIDGE_DDS)
  set(BRIDGE_BUILD_DDS 1)
endif()

set(BRIDGE_CVTE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/arm_control_sdk" CACHE PATH
  "Path to the CVTE arm_control_sdk package")
if(BUILD_ROBOT_CVTE_CARM)
  if(NOT EXISTS "${BRIDGE_CVTE_SDK_DIR}/lib/cmake/arm_control_sdk")
    message(FATAL_ERROR
      "CVTE CArm SDK not found at ${BRIDGE_CVTE_SDK_DIR}. Set BRIDGE_CVTE_SDK_DIR or disable BUILD_ROBOT_CVTE_CARM.")
  endif()
  list(APPEND CMAKE_PREFIX_PATH "${BRIDGE_CVTE_SDK_DIR}")
  find_package(arm_control_sdk CONFIG REQUIRED)
endif()

set(BRIDGE_PM01_REAL_BACKEND_AVAILABLE 0)

set(BRIDGE_UNITREE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/library/unitree_sdk2")
set(BRIDGE_CYCLONE_SDK_DIR "${BRIDGE_UNITREE_SDK_DIR}/thirdparty")
set(BRIDGE_TARGET_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
if(BRIDGE_TARGET_ARCH STREQUAL "arm64")
  set(BRIDGE_TARGET_ARCH "aarch64")
endif()
set(BRIDGE_CYCLONE_LIB_DIR "${BRIDGE_CYCLONE_SDK_DIR}/lib/${BRIDGE_TARGET_ARCH}")
set(BRIDGE_UNITREE_LIB_DIR "${BRIDGE_UNITREE_SDK_DIR}/lib/${BRIDGE_TARGET_ARCH}")
set(BRIDGE_UNITREE_SDK_STATIC_LIB "${BRIDGE_UNITREE_LIB_DIR}/libunitree_sdk2.a")
set(BRIDGE_UNITREE_SDK_FILTERED_LIB "${CMAKE_CURRENT_BINARY_DIR}/libunitree_sdk2_nomain.a")
if(BUILD_BRIDGE_DDS)
  set(CMAKE_BUILD_RPATH "${BRIDGE_CYCLONE_LIB_DIR}")
  set(CMAKE_INSTALL_RPATH "${BRIDGE_CYCLONE_LIB_DIR}")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_library(bridge_project_options INTERFACE)
target_compile_features(bridge_project_options INTERFACE cxx_std_17)

target_include_directories(bridge_project_options INTERFACE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_BINARY_DIR}/generated"
)

if(BUILD_BRIDGE_DDS)
  target_link_directories(bridge_project_options INTERFACE
    "${BRIDGE_CYCLONE_LIB_DIR}"
  )
  if(NOT APPLE)
    target_link_options(bridge_project_options INTERFACE
      "-Wl,--disable-new-dtags"
      "-Wl,-rpath,${BRIDGE_CYCLONE_LIB_DIR}"
    )
  endif()
endif()

find_package(Threads REQUIRED)
if(BUILD_ROBOT_PM01 OR BUILD_ROBOT_AGIBOT_X2)
  find_package(rclcpp QUIET)
endif()
if(BUILD_GAMEPAD_SDL)
  find_package(SDL2 QUIET CONFIG)
  if(NOT TARGET SDL2::SDL2)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(SDL2 QUIET IMPORTED_TARGET sdl2)
    endif()
  endif()
endif()

set(BRIDGE_BUILD_GAMEPAD_SDL 0)
if(BUILD_GAMEPAD_SDL AND (TARGET SDL2::SDL2 OR TARGET PkgConfig::SDL2))
  set(BRIDGE_BUILD_GAMEPAD_SDL 1)
endif()

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/include/bridge/build_config.hpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/generated/bridge/build_config.hpp"
  @ONLY
)

set(BRIDGE_INTERFACE_PROTOCOL_SOURCE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/interface_protocol")
set(BRIDGE_INTERFACE_PROTOCOL_BUILD_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/vendor/interface_protocol-build")
set(BRIDGE_INTERFACE_PROTOCOL_INSTALL_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/vendor/interface_protocol-install")

if(BUILD_ROBOT_PM01 AND EXISTS "${BRIDGE_INTERFACE_PROTOCOL_SOURCE_DIR}/CMakeLists.txt")
  file(GLOB BRIDGE_INTERFACE_PROTOCOL_SOURCE_FILES CONFIGURE_DEPENDS
    "${BRIDGE_INTERFACE_PROTOCOL_SOURCE_DIR}/CMakeLists.txt"
    "${BRIDGE_INTERFACE_PROTOCOL_SOURCE_DIR}/package.xml"
    "${BRIDGE_INTERFACE_PROTOCOL_SOURCE_DIR}/msg/*.msg")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${BRIDGE_INTERFACE_PROTOCOL_SOURCE_FILES})

  file(MAKE_DIRECTORY "${BRIDGE_INTERFACE_PROTOCOL_BUILD_DIR}")

  set(BRIDGE_INTERFACE_PROTOCOL_CONFIGURE_ARGS
    -S "${BRIDGE_INTERFACE_PROTOCOL_SOURCE_DIR}"
    -B "${BRIDGE_INTERFACE_PROTOCOL_BUILD_DIR}"
    -DCMAKE_INSTALL_PREFIX=${BRIDGE_INTERFACE_PROTOCOL_INSTALL_DIR}
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})

  if(CMAKE_GENERATOR)
    list(APPEND BRIDGE_INTERFACE_PROTOCOL_CONFIGURE_ARGS -G "${CMAKE_GENERATOR}")
  endif()
  if(CMAKE_GENERATOR_PLATFORM)
    list(APPEND BRIDGE_INTERFACE_PROTOCOL_CONFIGURE_ARGS -A "${CMAKE_GENERATOR_PLATFORM}")
  endif()
  if(CMAKE_GENERATOR_TOOLSET)
    list(APPEND BRIDGE_INTERFACE_PROTOCOL_CONFIGURE_ARGS -T "${CMAKE_GENERATOR_TOOLSET}")
  endif()
  if(CMAKE_TOOLCHAIN_FILE)
    list(APPEND BRIDGE_INTERFACE_PROTOCOL_CONFIGURE_ARGS
      -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE})
  endif()
  if(CMAKE_PREFIX_PATH)
    list(APPEND BRIDGE_INTERFACE_PROTOCOL_CONFIGURE_ARGS
      -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH})
  endif()
  if(CMAKE_MAKE_PROGRAM)
    list(APPEND BRIDGE_INTERFACE_PROTOCOL_CONFIGURE_ARGS
      -DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM})
  endif()

  set(BRIDGE_INTERFACE_PROTOCOL_BUILD_ARGS
    --build "${BRIDGE_INTERFACE_PROTOCOL_BUILD_DIR}"
    --target install)

  if(CMAKE_CONFIGURATION_TYPES)
    list(GET CMAKE_CONFIGURATION_TYPES 0 BRIDGE_INTERFACE_PROTOCOL_BUILD_CONFIG)
    list(APPEND BRIDGE_INTERFACE_PROTOCOL_BUILD_ARGS
      --config "${BRIDGE_INTERFACE_PROTOCOL_BUILD_CONFIG}")
  elseif(CMAKE_BUILD_TYPE)
    list(APPEND BRIDGE_INTERFACE_PROTOCOL_BUILD_ARGS
      --config "${CMAKE_BUILD_TYPE}")
  endif()

  execute_process(
    COMMAND ${CMAKE_COMMAND} ${BRIDGE_INTERFACE_PROTOCOL_CONFIGURE_ARGS}
    RESULT_VARIABLE BRIDGE_INTERFACE_PROTOCOL_CONFIGURE_RESULT
  )
  if(NOT BRIDGE_INTERFACE_PROTOCOL_CONFIGURE_RESULT EQUAL 0)
    message(FATAL_ERROR
      "Failed to configure vendored interface_protocol package from ${BRIDGE_INTERFACE_PROTOCOL_SOURCE_DIR}")
  endif()

  execute_process(
    COMMAND ${CMAKE_COMMAND} ${BRIDGE_INTERFACE_PROTOCOL_BUILD_ARGS}
    RESULT_VARIABLE BRIDGE_INTERFACE_PROTOCOL_BUILD_RESULT
  )
  if(NOT BRIDGE_INTERFACE_PROTOCOL_BUILD_RESULT EQUAL 0)
    message(FATAL_ERROR
      "Failed to build vendored interface_protocol package from ${BRIDGE_INTERFACE_PROTOCOL_SOURCE_DIR}")
  endif()

  set(interface_protocol_DIR
    "${BRIDGE_INTERFACE_PROTOCOL_INSTALL_DIR}/share/interface_protocol/cmake"
    CACHE PATH "Vendored interface_protocol package config directory" FORCE)
endif()

if(BUILD_ROBOT_PM01 AND DEFINED interface_protocol_DIR AND EXISTS "${interface_protocol_DIR}")
  find_package(interface_protocol QUIET PATHS "${interface_protocol_DIR}" NO_DEFAULT_PATH)
elseif(BUILD_ROBOT_PM01)
  find_package(interface_protocol QUIET)
endif()

if(BUILD_BRIDGE_DDS)
  if(NOT EXISTS "${BRIDGE_CYCLONE_LIB_DIR}/libddsc.so")
    message(FATAL_ERROR "Missing CycloneDDS library: ${BRIDGE_CYCLONE_LIB_DIR}/libddsc.so")
  endif()
  if(NOT EXISTS "${BRIDGE_CYCLONE_LIB_DIR}/libddscxx.so")
    message(FATAL_ERROR "Missing CycloneDDS library: ${BRIDGE_CYCLONE_LIB_DIR}/libddscxx.so")
  endif()
  if(NOT EXISTS "${BRIDGE_UNITREE_SDK_STATIC_LIB}")
    message(FATAL_ERROR "Missing Unitree SDK library: ${BRIDGE_UNITREE_SDK_STATIC_LIB}")
  endif()
endif()

if(BUILD_BRIDGE_DDS)
if(NOT TARGET ddsc)
  add_library(ddsc SHARED IMPORTED GLOBAL)
  set_target_properties(ddsc PROPERTIES
    IMPORTED_LOCATION "${BRIDGE_CYCLONE_LIB_DIR}/libddsc.so"
    IMPORTED_NO_SONAME TRUE)
  target_link_libraries(ddsc INTERFACE Threads::Threads)
  target_include_directories(ddsc INTERFACE
    "${BRIDGE_CYCLONE_SDK_DIR}/include")
endif()

if(NOT TARGET ddscxx)
  add_library(ddscxx SHARED IMPORTED GLOBAL)
  set_target_properties(ddscxx PROPERTIES
    IMPORTED_LOCATION "${BRIDGE_CYCLONE_LIB_DIR}/libddscxx.so"
    IMPORTED_NO_SONAME TRUE)
  target_link_libraries(ddscxx INTERFACE Threads::Threads)
  target_include_directories(ddscxx INTERFACE
    "${BRIDGE_CYCLONE_SDK_DIR}/include"
    "${BRIDGE_CYCLONE_SDK_DIR}/include/ddscxx")
endif()

add_custom_command(
  OUTPUT "${BRIDGE_UNITREE_SDK_FILTERED_LIB}"
  COMMAND ${CMAKE_COMMAND} -E copy "${BRIDGE_UNITREE_SDK_STATIC_LIB}" "${BRIDGE_UNITREE_SDK_FILTERED_LIB}"
  COMMAND ${CMAKE_COMMAND} -E env
    "BRIDGE_AR=${CMAKE_AR}"
    "BRIDGE_ARCHIVE=${BRIDGE_UNITREE_SDK_FILTERED_LIB}"
    ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/remove_archive_entry.cmake"
  COMMAND ${CMAKE_RANLIB} "${BRIDGE_UNITREE_SDK_FILTERED_LIB}"
  DEPENDS "${BRIDGE_UNITREE_SDK_STATIC_LIB}"
  COMMENT "Creating libunitree_sdk2_nomain.a"
  VERBATIM)

add_custom_target(unitree_sdk2_nomain ALL DEPENDS "${BRIDGE_UNITREE_SDK_FILTERED_LIB}")

add_library(bridge_cyclonedds_options INTERFACE)
target_link_libraries(bridge_cyclonedds_options INTERFACE
  bridge_project_options
  "${BRIDGE_UNITREE_SDK_FILTERED_LIB}"
  ddscxx
  ddsc)
target_include_directories(bridge_cyclonedds_options INTERFACE
  "${BRIDGE_UNITREE_SDK_DIR}/include"
  "${BRIDGE_CYCLONE_SDK_DIR}/include"
  "${BRIDGE_CYCLONE_SDK_DIR}/include/ddscxx")
endif()

add_library(bridge_gamepad STATIC
  src/bridge/gamepad/common_gamepad.cpp
)

if(BUILD_ROBOT_AGIBOT_X2)
  find_package(aimdk_msgs QUIET)
  # Agibot's deployed image ships aimdk_msgs as a prebuilt ROS 2 package
  # without a CMake package config. Allow that layout in addition to a normal
  # ament installation.
  if(NOT aimdk_msgs_FOUND)
    find_path(BRIDGE_AIMDK_MSGS_INCLUDE_DIR
      NAMES aimdk_msgs/msg/joint_command_array.hpp
      PATHS
        "${BRIDGE_AIMDK_MSGS_PREFIX}/include/aimdk_msgs"
        "/agibot/software/common/include"
        "/agibot/software/aimdk/src/aimdk_msgs/prebuilt_aarch64/include/aimdk_msgs"
      NO_DEFAULT_PATH)
    find_library(BRIDGE_AIMDK_MSGS_TYPESUPPORT_CPP
      NAMES aimdk_msgs__rosidl_typesupport_cpp
      PATHS
        "${BRIDGE_AIMDK_MSGS_PREFIX}/lib"
        "/agibot/software/common/lib"
        "/agibot/software/aimdk/src/aimdk_msgs/prebuilt_aarch64/lib"
      NO_DEFAULT_PATH)
    if(BRIDGE_AIMDK_MSGS_INCLUDE_DIR AND BRIDGE_AIMDK_MSGS_TYPESUPPORT_CPP)
      add_library(bridge_aimdk_msgs INTERFACE)
      target_include_directories(bridge_aimdk_msgs INTERFACE
        "${BRIDGE_AIMDK_MSGS_INCLUDE_DIR}")
      target_link_libraries(bridge_aimdk_msgs INTERFACE
        "${BRIDGE_AIMDK_MSGS_TYPESUPPORT_CPP}")
      set(BRIDGE_AGIBOT_X2_AIMDK_MSGS_AVAILABLE ON)
    endif()
  else()
    set(BRIDGE_AGIBOT_X2_AIMDK_MSGS_AVAILABLE ON)
  endif()
endif()
target_link_libraries(bridge_gamepad PUBLIC bridge_project_options)

if(BUILD_GAMEPAD_SDL AND (TARGET SDL2::SDL2 OR TARGET PkgConfig::SDL2))
  add_library(bridge_gamepad_sdl STATIC
    src/bridge/gamepad/sdl_gamepad.cpp
  )
  target_link_libraries(bridge_gamepad_sdl PUBLIC bridge_gamepad)
  if(TARGET SDL2::SDL2)
    target_link_libraries(bridge_gamepad_sdl PUBLIC SDL2::SDL2)
  else()
    target_link_libraries(bridge_gamepad_sdl PUBLIC PkgConfig::SDL2)
  endif()
endif()

add_library(bridge_core STATIC
  src/bridge/core/bridge_runner.cpp
)
target_link_libraries(bridge_core PUBLIC bridge_project_options bridge_gamepad)

add_library(bridge_config STATIC
  src/bridge/config/bridge_config.cpp
  src/bridge/config/joint_map.cpp
)
target_link_libraries(bridge_config PUBLIC bridge_project_options bridge_core)

if(BUILD_BRIDGE_DDS)
add_library(bridge_dds_codecs STATIC
  src/bridge/dds/dds_translation.cpp
)
target_link_libraries(bridge_dds_codecs PUBLIC bridge_cyclonedds_options bridge_core)
add_dependencies(bridge_dds_codecs unitree_sdk2_nomain)

add_library(bridge_dds_runtime STATIC
  src/bridge/dds/dds_runtime.cpp
  src/bridge/dds/dds_factory.cpp
  src/bridge/dds/dds_command_ingress.cpp
  src/bridge/dds/dds_state_egress.cpp
)
target_link_libraries(bridge_dds_runtime PUBLIC
  bridge_cyclonedds_options
  bridge_core
  bridge_config
  bridge_dds_codecs
)
add_dependencies(bridge_dds_runtime unitree_sdk2_nomain)
endif()

if(BUILD_ROBOT_KEPLER_K2)
  add_library(bridge_adapter_kepler_k2 STATIC
    src/bridge/adapters/kepler_k2/kepler_k2_adapter.cpp
  )
  target_link_libraries(bridge_adapter_kepler_k2 PUBLIC bridge_project_options)
endif()

if(BUILD_ROBOT_PM01)
  add_library(bridge_adapter_pm01 STATIC
    src/bridge/adapters/pm01/pm01_adapter.cpp
  )
  target_link_libraries(bridge_adapter_pm01 PUBLIC bridge_project_options)

  if(rclcpp_FOUND AND interface_protocol_FOUND)
    set(BRIDGE_PM01_REAL_BACKEND_AVAILABLE 1)
    configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/include/bridge/build_config.hpp.in"
      "${CMAKE_CURRENT_BINARY_DIR}/generated/bridge/build_config.hpp"
      @ONLY
    )
    if(TARGET rclcpp::rclcpp)
      target_link_libraries(bridge_adapter_pm01 PUBLIC rclcpp::rclcpp)
    elseif(DEFINED rclcpp_LIBRARIES)
      target_link_libraries(bridge_adapter_pm01 PUBLIC ${rclcpp_LIBRARIES})
    endif()
    if(DEFINED rclcpp_INCLUDE_DIRS)
      target_include_directories(bridge_adapter_pm01 PUBLIC ${rclcpp_INCLUDE_DIRS})
    endif()
    if(DEFINED interface_protocol_INCLUDE_DIRS)
      target_include_directories(bridge_adapter_pm01 PUBLIC ${interface_protocol_INCLUDE_DIRS})
    endif()
    if(DEFINED interface_protocol_LIBRARIES)
      target_link_libraries(bridge_adapter_pm01 PUBLIC ${interface_protocol_LIBRARIES})
    endif()
    foreach(interface_target
        interface_protocol::interface_protocol__rosidl_generator_cpp
        interface_protocol::interface_protocol__rosidl_typesupport_cpp
        interface_protocol::interface_protocol__rosidl_typesupport_introspection_cpp)
      if(TARGET ${interface_target})
        target_link_libraries(bridge_adapter_pm01 PUBLIC ${interface_target})
      endif()
    endforeach()
  endif()
endif()

if(BUILD_ROBOT_UNITREE_G1)
  add_library(bridge_adapter_unitree_g1 STATIC
    src/bridge/adapters/unitree_g1/unitree_g1_low_level_protocol.cpp
    src/bridge/adapters/unitree_g1/unitree_g1_adapter.cpp
    src/bridge/adapters/unitree_g1/unitree_g1_sdk.cpp
    src/bridge/adapters/unitree_g1/unitree_g1_sensor_session.cpp
    src/bridge/adapters/unitree_g1/unitree_g1_low_level_session.cpp
  )
  target_link_libraries(bridge_adapter_unitree_g1 PUBLIC
    bridge_project_options
    bridge_cyclonedds_options)
  add_dependencies(bridge_adapter_unitree_g1 unitree_sdk2_nomain)

  if(BUILD_TESTING)
    add_executable(unitree_g1_motion_control_test tests/demo/unitree_g1_motion_control_test.cpp)
    add_test(NAME unitree_g1_motion_control_test COMMAND unitree_g1_motion_control_test)
  endif()

  if(BUILD_UNITREE_G1_MOTION_DEMO)
    add_executable(unitree_g1_motion_demo
      demo/unitree_g1_motion_demo.cpp
    )
    target_link_libraries(unitree_g1_motion_demo PRIVATE
      bridge_project_options
      bridge_adapter_unitree_g1
    )
    set_target_properties(unitree_g1_motion_demo PROPERTIES
      BUILD_RPATH "${BRIDGE_CYCLONE_LIB_DIR}"
    )
    add_dependencies(unitree_g1_motion_demo unitree_sdk2_nomain)
    if(BUILD_TESTING)
      find_package(Python3 QUIET COMPONENTS Interpreter)
      if(Python3_Interpreter_FOUND)
        add_test(NAME unitree_g1_motion_terminal_test
          COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/demo/unitree_g1_motion_terminal_test.py"
            "$<TARGET_FILE:unitree_g1_motion_demo>")
        set_tests_properties(unitree_g1_motion_terminal_test PROPERTIES TIMEOUT 60)
      endif()
    endif()
  endif()

  if(BUILD_UNITREE_G1_HARDWARE_TESTS)
    add_executable(unitree_g1_sdk_check
      tests/adapters/unitree_g1_sdk_check.cpp
    )
    target_link_libraries(unitree_g1_sdk_check PRIVATE
      bridge_project_options
      bridge_adapter_unitree_g1
    )
    add_dependencies(unitree_g1_sdk_check unitree_sdk2_nomain)

    add_executable(unitree_g1_services_check
      tests/adapters/unitree_g1_services_check.cpp
    )
    target_link_libraries(unitree_g1_services_check PRIVATE
      bridge_project_options
      bridge_adapter_unitree_g1
    )
    add_dependencies(unitree_g1_services_check unitree_sdk2_nomain)

    add_executable(unitree_g1_sdk_robot_smoke
      tests/adapters/unitree_g1_sdk_robot_smoke.cpp
    )
    target_link_libraries(unitree_g1_sdk_robot_smoke PRIVATE
      bridge_project_options
      bridge_adapter_unitree_g1
    )
    add_dependencies(unitree_g1_sdk_robot_smoke unitree_sdk2_nomain)

    add_executable(unitree_g1_robot_services_smoke
      tests/adapters/unitree_g1_robot_services_smoke.cpp
    )
    target_link_libraries(unitree_g1_robot_services_smoke PRIVATE
      bridge_project_options
      bridge_adapter_unitree_g1
    )
    add_dependencies(unitree_g1_robot_services_smoke unitree_sdk2_nomain)
  endif()

  add_executable(unitree_g1_sensor_reader
    examples/unitree_g1/unitree_g1_sensor_reader.cpp
  )
  target_link_libraries(unitree_g1_sensor_reader PRIVATE bridge_adapter_unitree_g1)
  add_dependencies(unitree_g1_sensor_reader unitree_sdk2_nomain)
  if(BUILD_TESTING)
    add_test(NAME unitree_g1_sensor_reader_help COMMAND unitree_g1_sensor_reader --help)
    add_test(NAME unitree_g1_sensor_reader_invalid_args
      COMMAND unitree_g1_sensor_reader --duration invalid)
    set_tests_properties(unitree_g1_sensor_reader_invalid_args PROPERTIES WILL_FAIL TRUE)
  endif()
endif()

if(BUILD_ROBOT_CVTE_CARM)
  add_library(bridge_adapter_cvte_carm STATIC
    src/bridge/adapters/cvte_carm/cvte_carm_adapter.cpp
  )
  target_link_libraries(bridge_adapter_cvte_carm PUBLIC
    bridge_project_options
    ${arm_control_sdk_LIBRARIES}
  )
  target_include_directories(bridge_adapter_cvte_carm PUBLIC
    "${arm_control_sdk_INCLUDE_DIRS}"
  )
  target_link_directories(bridge_adapter_cvte_carm PUBLIC
    "${BRIDGE_CVTE_SDK_DIR}/lib"
  )
  set_target_properties(bridge_adapter_cvte_carm PROPERTIES
    BUILD_RPATH "${BRIDGE_CVTE_SDK_DIR}/lib"
    INSTALL_RPATH "${BRIDGE_CVTE_SDK_DIR}/lib"
  )

  add_executable(cvte_carm_sdk_smoke
    tests/adapters/cvte_carm_sdk_smoke.cpp
  )
  target_link_libraries(cvte_carm_sdk_smoke PRIVATE
    bridge_project_options
    bridge_adapter_cvte_carm
  )
  set_target_properties(cvte_carm_sdk_smoke PROPERTIES
    BUILD_RPATH "${BRIDGE_CVTE_SDK_DIR}/lib"
  )

  add_executable(cvte_carm_check
    tests/adapters/cvte_carm_check.cpp
  )
  target_link_libraries(cvte_carm_check PRIVATE
    bridge_project_options
    bridge_adapter_cvte_carm
  )
  set_target_properties(cvte_carm_check PROPERTIES
    BUILD_RPATH "${BRIDGE_CVTE_SDK_DIR}/lib"
  )
endif()

if(BUILD_ROBOT_AGIBOT_X2)
  add_library(bridge_adapter_agibot_x2 STATIC
    src/bridge/adapters/agibot_x2/agibot_x2_adapter.cpp
  )
  target_link_libraries(bridge_adapter_agibot_x2 PUBLIC bridge_project_options)
  if(rclcpp_FOUND AND BRIDGE_AGIBOT_X2_AIMDK_MSGS_AVAILABLE)
    target_compile_definitions(bridge_adapter_agibot_x2 PRIVATE BRIDGE_AGIBOT_X2_ROS2_AVAILABLE=1)
    if(TARGET rclcpp::rclcpp)
      target_link_libraries(bridge_adapter_agibot_x2 PUBLIC rclcpp::rclcpp)
    endif()
    if(TARGET aimdk_msgs::aimdk_msgs__rosidl_generator_cpp)
      target_link_libraries(bridge_adapter_agibot_x2 PUBLIC
        aimdk_msgs::aimdk_msgs__rosidl_generator_cpp)
    elseif(TARGET aimdk_msgs::aimdk_msgs__rosidl_typesupport_cpp)
      target_link_libraries(bridge_adapter_agibot_x2 PUBLIC
        aimdk_msgs::aimdk_msgs__rosidl_typesupport_cpp)
    elseif(TARGET bridge_aimdk_msgs)
      target_link_libraries(bridge_adapter_agibot_x2 PUBLIC bridge_aimdk_msgs)
    endif()
  endif()
  if(BUILD_TESTING)
    add_executable(agibot_x2_adapter_test
      tests/adapters/agibot_x2_adapter_test.cpp)
    target_link_libraries(agibot_x2_adapter_test PRIVATE
      bridge_project_options bridge_adapter_agibot_x2)
    add_test(NAME agibot_x2_adapter_test COMMAND agibot_x2_adapter_test)
  endif()
  add_executable(agibot_x2_state_probe
    tests/adapters/agibot_x2_state_probe.cpp)
  target_link_libraries(agibot_x2_state_probe PRIVATE
    bridge_project_options bridge_adapter_agibot_x2)
endif()

add_library(gamepad_demo_support STATIC
  demo/unitree_g1_gamepad.cpp
  demo/gamepad_table.cpp
)
target_link_libraries(gamepad_demo_support PUBLIC
  bridge_project_options
  bridge_gamepad
)
if(TARGET bridge_gamepad_sdl)
  target_link_libraries(gamepad_demo_support PUBLIC bridge_gamepad_sdl)
endif()

add_executable(common_gamepad_wireless_codec_demo
  demo/common_gamepad_wireless_codec_demo.cpp
)
target_link_libraries(common_gamepad_wireless_codec_demo PRIVATE
  bridge_project_options
  bridge_gamepad
)

add_library(bridge_extensions STATIC
  src/bridge/extensions/legacy_kepler_hooks.cpp
  src/bridge/extensions/terminal_observation_hook.cpp
)
target_link_libraries(bridge_extensions PUBLIC bridge_project_options bridge_core)

if(BUILD_BRIDGE_DDS)
  add_library(bridge_app_support STATIC
    src/bridge/app/robot_assembly_factory.cpp
  )
  target_link_libraries(bridge_app_support PUBLIC
    bridge_project_options
    bridge_core
    bridge_dds_codecs
    bridge_dds_runtime
    bridge_config
  )
  if(BUILD_ROBOT_KEPLER_K2)
    target_link_libraries(bridge_app_support PUBLIC bridge_adapter_kepler_k2)
  endif()
  if(BUILD_ROBOT_PM01)
    target_link_libraries(bridge_app_support PUBLIC bridge_adapter_pm01)
  endif()
  if(BUILD_ROBOT_UNITREE_G1)
    target_link_libraries(bridge_app_support PUBLIC bridge_adapter_unitree_g1)
  endif()
  if(BUILD_ROBOT_CVTE_CARM)
    target_link_libraries(bridge_app_support PUBLIC bridge_adapter_cvte_carm)
  endif()
  if(BUILD_ROBOT_AGIBOT_X2)
    target_link_libraries(bridge_app_support PUBLIC bridge_adapter_agibot_x2)
  endif()
  if(TARGET bridge_gamepad_sdl)
    target_link_libraries(bridge_app_support PUBLIC bridge_gamepad_sdl)
  endif()

  add_executable(adapter_bridge
    src/apps/adapter_bridge_main.cpp
  )
  target_link_libraries(adapter_bridge PRIVATE
    bridge_project_options
    bridge_core
    bridge_config
    bridge_app_support
    bridge_extensions
  )
  set_target_properties(adapter_bridge PROPERTIES
    BUILD_RPATH "${BRIDGE_CYCLONE_LIB_DIR}"
  )
endif()

if(BUILD_ROBOT_UNITREE_G1)
  add_executable(fake_state_publisher
    src/apps/fake_state_publisher.cpp
  )
  target_link_libraries(fake_state_publisher PRIVATE
    bridge_cyclonedds_options
    bridge_config
  )
  set_target_properties(fake_state_publisher PROPERTIES
    BUILD_RPATH "${BRIDGE_CYCLONE_LIB_DIR}"
  )
  add_dependencies(fake_state_publisher unitree_sdk2_nomain)
  add_test(NAME fake_state_publisher_help_test COMMAND fake_state_publisher --help)
  set_tests_properties(fake_state_publisher_help_test PROPERTIES
    PASS_REGULAR_EXPRESSION "custom/mujoco/rt/lowstate.*rt/secondary_imu"
  )

  add_executable(state_receiver
    src/apps/state_receiver.cpp
  )
  target_link_libraries(state_receiver PRIVATE
    bridge_cyclonedds_options
    bridge_config
    gamepad_demo_support
  )
  set_target_properties(state_receiver PROPERTIES
    BUILD_RPATH "${BRIDGE_CYCLONE_LIB_DIR}"
  )
  add_dependencies(state_receiver unitree_sdk2_nomain)
  add_test(NAME state_receiver_help_test COMMAND state_receiver --help)
  set_tests_properties(state_receiver_help_test PROPERTIES
    PASS_REGULAR_EXPRESSION "Prints the latest state at 50 Hz"
  )

  add_executable(lowstate_frequency_receiver
    demo/lowstate_frequency_receiver.cpp
  )
  target_link_libraries(lowstate_frequency_receiver PRIVATE
    bridge_cyclonedds_options
    bridge_config
  )
  set_target_properties(lowstate_frequency_receiver PROPERTIES
    BUILD_RPATH "${BRIDGE_CYCLONE_LIB_DIR}"
  )
  add_dependencies(lowstate_frequency_receiver unitree_sdk2_nomain)
  add_test(NAME lowstate_frequency_receiver_help_test COMMAND lowstate_frequency_receiver --help)
  set_tests_properties(lowstate_frequency_receiver_help_test PROPERTIES
    PASS_REGULAR_EXPRESSION "Prints received LowState frequency"
  )

  add_executable(lowstate_field_recorder
    demo/lowstate_field_recorder.cpp
  )
  target_link_libraries(lowstate_field_recorder PRIVATE
    bridge_cyclonedds_options
    bridge_config
  )
  set_target_properties(lowstate_field_recorder PROPERTIES
    BUILD_RPATH "${BRIDGE_CYCLONE_LIB_DIR}"
  )
  add_dependencies(lowstate_field_recorder unitree_sdk2_nomain)
  add_test(NAME lowstate_field_recorder_help_test COMMAND lowstate_field_recorder --help)
  set_tests_properties(lowstate_field_recorder_help_test PROPERTIES
    PASS_REGULAR_EXPRESSION "Records Unitree LowState fields before and after forwarding"
  )

  add_executable(gamepad_dds_receiver
    demo/gamepad_dds_receiver.cpp
  )
  target_link_libraries(gamepad_dds_receiver PRIVATE
    bridge_cyclonedds_options
    bridge_config
    gamepad_demo_support
  )
  set_target_properties(gamepad_dds_receiver PROPERTIES
    BUILD_RPATH "${BRIDGE_CYCLONE_LIB_DIR}"
  )
  add_dependencies(gamepad_dds_receiver unitree_sdk2_nomain)

  if(TARGET bridge_gamepad_sdl)
    add_executable(gamepad_dds_sender
      demo/gamepad_dds_sender.cpp
    )
    target_link_libraries(gamepad_dds_sender PRIVATE
      bridge_cyclonedds_options
      bridge_config
      bridge_gamepad_sdl
      gamepad_demo_support
    )
    set_target_properties(gamepad_dds_sender PROPERTIES
      BUILD_RPATH "${BRIDGE_CYCLONE_LIB_DIR}"
    )
    add_dependencies(gamepad_dds_sender unitree_sdk2_nomain)
  endif()
endif()

if(BUILD_TESTING AND BUILD_BRIDGE_DDS)
  function(bridge_add_test target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} PRIVATE ${ARGN})
    set_target_properties(${target_name} PROPERTIES
      BUILD_RPATH "${BRIDGE_CYCLONE_LIB_DIR}"
    )
    add_test(NAME ${target_name} COMMAND ${target_name})
  endfunction()

  bridge_add_test(bridge_runner_test
    tests/core/bridge_runner_test.cpp
    bridge_project_options
    bridge_core
    bridge_config
    bridge_dds_codecs
  )

  bridge_add_test(runner_frequency_log_test
    tests/core/runner_frequency_log_test.cpp
    bridge_project_options
  )

  bridge_add_test(dds_state_egress_frequency_test
    tests/dds/dds_state_egress_frequency_test.cpp
    bridge_project_options
  )

  bridge_add_test(forwarding_stats_test
    tests/observability/forwarding_stats_test.cpp
    bridge_project_options
  )

  bridge_add_test(fake_adapter_compile_test
    tests/core/fake_adapter_compile_test.cpp
    bridge_project_options
  )

  bridge_add_test(joint_map_test
    tests/config/joint_map_test.cpp
    bridge_project_options
    bridge_config
  )

  bridge_add_test(bridge_config_test
    tests/config/bridge_config_test.cpp
    bridge_project_options
    bridge_config
  )

  bridge_add_test(g1_bridge_config_test
    tests/config/g1_bridge_config_test.cpp
    bridge_project_options
    bridge_config
  )

  bridge_add_test(common_gamepad_test
    tests/gamepad/common_gamepad_test.cpp
    bridge_project_options
    bridge_gamepad
  )

  if(TARGET bridge_gamepad_sdl)
    bridge_add_test(sdl_gamepad_test
      tests/gamepad/sdl_gamepad_test.cpp
      bridge_project_options
      bridge_gamepad_sdl
    )
  endif()

  bridge_add_test(fake_transport_test
    tests/dds/fake_transport_test.cpp
    bridge_cyclonedds_options
    bridge_core
    bridge_config
    bridge_dds_codecs
    bridge_dds_runtime
  )
  add_dependencies(fake_transport_test unitree_sdk2_nomain)

  bridge_add_test(generated_header_location_test
    tests/dds/generated_header_location_test.cpp
    bridge_cyclonedds_options
  )
  add_dependencies(generated_header_location_test unitree_sdk2_nomain)

  bridge_add_test(dds_translation_test
    tests/dds/dds_translation_test.cpp
    bridge_cyclonedds_options
    bridge_config
    bridge_dds_codecs
  )
  add_dependencies(dds_translation_test unitree_sdk2_nomain)

  if(BUILD_ROBOT_KEPLER_K2)
    bridge_add_test(kepler_k2_adapter_test
      tests/adapters/kepler_k2_adapter_test.cpp
      bridge_project_options
      bridge_adapter_kepler_k2
    )
  endif()

  if(BUILD_ROBOT_PM01)
    bridge_add_test(pm01_adapter_test
      tests/adapters/pm01_adapter_test.cpp
      bridge_project_options
      bridge_adapter_pm01
    )
    if(rclcpp_FOUND AND interface_protocol_FOUND)
      if(TARGET rclcpp::rclcpp)
        target_link_libraries(pm01_adapter_test PRIVATE rclcpp::rclcpp)
      elseif(DEFINED rclcpp_LIBRARIES)
        target_link_libraries(pm01_adapter_test PRIVATE ${rclcpp_LIBRARIES})
      endif()
      if(DEFINED rclcpp_INCLUDE_DIRS)
        target_include_directories(pm01_adapter_test PRIVATE ${rclcpp_INCLUDE_DIRS})
      endif()
      if(DEFINED interface_protocol_INCLUDE_DIRS)
        target_include_directories(pm01_adapter_test PRIVATE ${interface_protocol_INCLUDE_DIRS})
      endif()
      if(DEFINED interface_protocol_LIBRARIES)
        target_link_libraries(pm01_adapter_test PRIVATE ${interface_protocol_LIBRARIES})
      endif()
      foreach(interface_target
          interface_protocol::interface_protocol__rosidl_generator_cpp
          interface_protocol::interface_protocol__rosidl_typesupport_cpp
          interface_protocol::interface_protocol__rosidl_typesupport_introspection_cpp)
        if(TARGET ${interface_target})
          target_link_libraries(pm01_adapter_test PRIVATE ${interface_target})
        endif()
      endforeach()
  endif()

endif()

  if(BUILD_ROBOT_UNITREE_G1)
    bridge_add_test(unitree_g1_adapter_test
      tests/adapters/unitree_g1_adapter_test.cpp
      bridge_project_options
      bridge_adapter_unitree_g1
    )

    bridge_add_test(unitree_g1_sdk_test
      tests/adapters/unitree_g1_sdk_test.cpp
      bridge_project_options
      bridge_adapter_unitree_g1
    )

    bridge_add_test(unitree_g1_sensor_session_test
      tests/adapters/unitree_g1_sensor_session_test.cpp
      bridge_project_options
      bridge_adapter_unitree_g1
    )

    bridge_add_test(unitree_g1_low_level_session_test
      tests/adapters/unitree_g1_low_level_session_test.cpp
      bridge_project_options
      bridge_adapter_unitree_g1
    )

    bridge_add_test(unitree_g1_low_level_protocol_test
      tests/adapters/unitree_g1_low_level_protocol_test.cpp
      bridge_project_options
      bridge_adapter_unitree_g1
    )

    bridge_add_test(unitree_g1_gamepad_test
      tests/adapters/unitree_g1_gamepad_test.cpp
      bridge_project_options
      gamepad_demo_support
      bridge_gamepad
    )
  endif()

  bridge_add_test(hooks_isolation_test
    tests/extensions/hooks_isolation_test.cpp
    bridge_project_options
    bridge_extensions
  )

  bridge_add_test(gamepad_table_test
    tests/demo/gamepad_table_test.cpp
    bridge_project_options
    gamepad_demo_support
  )

  bridge_add_test(robot_assembly_factory_test
    tests/apps/robot_assembly_factory_test.cpp
    bridge_project_options
    bridge_core
    bridge_config
    bridge_app_support
    bridge_extensions
  )
  target_compile_definitions(robot_assembly_factory_test PRIVATE
    BRIDGE_REPO_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  if(BUILD_ROBOT_UNITREE_G1)
    bridge_add_test(g1_robot_assembly_test
      tests/apps/g1_robot_assembly_test.cpp
      bridge_project_options
      bridge_core
      bridge_config
      bridge_app_support
      bridge_extensions
    )
    target_compile_definitions(g1_robot_assembly_test PRIVATE
      BRIDGE_REPO_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
    )

    bridge_add_test(fake_g1_state_pipeline_test
      tests/apps/fake_g1_state_pipeline_test.cpp
      bridge_project_options
      bridge_adapter_unitree_g1
      bridge_config
      bridge_dds_codecs
    )
  endif()

endif()
