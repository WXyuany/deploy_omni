cmake_minimum_required(VERSION 3.16)

project(adapter_any_robot LANGUAGES CXX)

option(BUILD_ROBOT_KEPLER_K2 "Build kepler_k2 adapter support" ON)
option(BUILD_ROBOT_PM01 "Build pm01 adapter support" ON)

if(NOT BUILD_ROBOT_KEPLER_K2 AND NOT BUILD_ROBOT_PM01)
  message(FATAL_ERROR "At least one robot adapter must be enabled. Set BUILD_ROBOT_KEPLER_K2 or BUILD_ROBOT_PM01 to ON.")
endif()

set(BRIDGE_UNITREE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/library/unitree_sdk2")
set(BRIDGE_CYCLONE_SDK_DIR "${BRIDGE_UNITREE_SDK_DIR}/thirdparty")
set(BRIDGE_CYCLONE_LIB_DIR "${BRIDGE_CYCLONE_SDK_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}")
set(BRIDGE_UNITREE_LIB_DIR "${BRIDGE_UNITREE_SDK_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}")
set(BRIDGE_UNITREE_SDK_STATIC_LIB "${BRIDGE_UNITREE_LIB_DIR}/libunitree_sdk2.a")
set(BRIDGE_UNITREE_SDK_FILTERED_LIB "${CMAKE_CURRENT_BINARY_DIR}/libunitree_sdk2_nomain.a")
set(CMAKE_BUILD_RPATH "${BRIDGE_CYCLONE_LIB_DIR}")
set(CMAKE_INSTALL_RPATH "${BRIDGE_CYCLONE_LIB_DIR}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_library(bridge_project_options INTERFACE)
target_compile_features(bridge_project_options INTERFACE cxx_std_17)
target_include_directories(bridge_project_options INTERFACE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_directories(bridge_project_options INTERFACE
  "${BRIDGE_CYCLONE_LIB_DIR}"
)
target_link_options(bridge_project_options INTERFACE
  "-Wl,--disable-new-dtags"
  "-Wl,-rpath,${BRIDGE_CYCLONE_LIB_DIR}"
)

find_package(Threads REQUIRED)
find_package(rclcpp QUIET)
find_package(interface_protocol QUIET)

if(NOT EXISTS "${BRIDGE_CYCLONE_LIB_DIR}/libddsc.so")
  message(FATAL_ERROR "Missing CycloneDDS library: ${BRIDGE_CYCLONE_LIB_DIR}/libddsc.so")
endif()
if(NOT EXISTS "${BRIDGE_CYCLONE_LIB_DIR}/libddscxx.so")
  message(FATAL_ERROR "Missing CycloneDDS library: ${BRIDGE_CYCLONE_LIB_DIR}/libddscxx.so")
endif()
if(NOT EXISTS "${BRIDGE_UNITREE_SDK_STATIC_LIB}")
  message(FATAL_ERROR "Missing Unitree SDK library: ${BRIDGE_UNITREE_SDK_STATIC_LIB}")
endif()

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
  ddscxx
  ddsc
  "${BRIDGE_UNITREE_SDK_FILTERED_LIB}")
target_include_directories(bridge_cyclonedds_options INTERFACE
  "${BRIDGE_UNITREE_SDK_DIR}/include"
  "${BRIDGE_CYCLONE_SDK_DIR}/include"
  "${BRIDGE_CYCLONE_SDK_DIR}/include/ddscxx")

add_library(bridge_core STATIC
  src/bridge/core/bridge_runner.cpp
)
target_link_libraries(bridge_core PUBLIC bridge_project_options)

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

add_library(bridge_config STATIC
  src/bridge/config/bridge_config.cpp
  src/bridge/config/joint_map.cpp
)
target_link_libraries(bridge_config PUBLIC bridge_project_options bridge_core)

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
    target_compile_definitions(bridge_adapter_pm01 PRIVATE BRIDGE_PM01_REAL_BACKEND_AVAILABLE=1)
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

add_library(bridge_extensions STATIC
  src/bridge/extensions/legacy_kepler_hooks.cpp
)
target_link_libraries(bridge_extensions PUBLIC bridge_project_options bridge_core)

add_library(bridge_app_support STATIC
  src/bridge/app/robot_assembly_factory.cpp
)
if(BUILD_ROBOT_KEPLER_K2)
  target_compile_definitions(bridge_app_support PUBLIC BRIDGE_BUILD_KEPLER_K2=1)
endif()
if(BUILD_ROBOT_PM01)
  target_compile_definitions(bridge_app_support PUBLIC BRIDGE_BUILD_PM01=1)
endif()
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

add_executable(kepler_k2_bridge
  src/apps/kepler_k2_bridge_main.cpp
)
target_link_libraries(kepler_k2_bridge PRIVATE
  bridge_project_options
  bridge_core
  bridge_config
  bridge_app_support
)
set_target_properties(kepler_k2_bridge PROPERTIES
  BUILD_RPATH "${BRIDGE_CYCLONE_LIB_DIR}"
)

if(BUILD_TESTING)
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
      target_compile_definitions(pm01_adapter_test PRIVATE BRIDGE_PM01_REAL_BACKEND_AVAILABLE=1)
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

  bridge_add_test(hooks_isolation_test
    tests/extensions/hooks_isolation_test.cpp
    bridge_project_options
    bridge_extensions
  )

  bridge_add_test(robot_assembly_factory_test
    tests/apps/robot_assembly_factory_test.cpp
    bridge_project_options
    bridge_core
    bridge_config
    bridge_app_support
  )
  target_compile_definitions(robot_assembly_factory_test PRIVATE
    BRIDGE_REPO_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
  )

endif()
