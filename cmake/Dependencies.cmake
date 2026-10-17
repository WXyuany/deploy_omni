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
  target_link_options(bridge_project_options INTERFACE
    "-Wl,--disable-new-dtags"
    "-Wl,-rpath,${BRIDGE_CYCLONE_LIB_DIR}"
  )
endif()

find_package(Threads REQUIRED)
if(BUILD_ROBOT_PM01)
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
  COMMAND ${CMAKE_AR} d "${BRIDGE_UNITREE_SDK_FILTERED_LIB}" service_main.cpp.o
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
