cmake_minimum_required(VERSION 3.16)

project(adapter_any_robot LANGUAGES CXX)

set(BRIDGE_FASTCDR_INCLUDE_DIR "/home/xsc/Fast-DDS/install/fastcdr/include")
set(BRIDGE_FASTDDS_INCLUDE_DIR "/home/xsc/Fast-DDS/install/fastdds/include")
set(BRIDGE_FASTDDS_LIB_DIR "/home/xsc/Fast-DDS/install/fastdds/lib")
set(BRIDGE_FASTCDR_LIB_DIR "/home/xsc/Fast-DDS/install/fastcdr/lib")
set(CMAKE_BUILD_RPATH "${BRIDGE_FASTDDS_LIB_DIR};${BRIDGE_FASTCDR_LIB_DIR}")
set(CMAKE_INSTALL_RPATH "${BRIDGE_FASTDDS_LIB_DIR};${BRIDGE_FASTCDR_LIB_DIR}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_library(bridge_project_options INTERFACE)
target_compile_features(bridge_project_options INTERFACE cxx_std_20)
target_include_directories(bridge_project_options INTERFACE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${BRIDGE_FASTCDR_INCLUDE_DIR}"
  "${BRIDGE_FASTDDS_INCLUDE_DIR}"
)

target_link_directories(bridge_project_options INTERFACE
  "${BRIDGE_FASTDDS_LIB_DIR}"
  "${BRIDGE_FASTCDR_LIB_DIR}"
)
target_link_options(bridge_project_options INTERFACE
  "-Wl,--disable-new-dtags"
  "-Wl,-rpath,${BRIDGE_FASTDDS_LIB_DIR}"
  "-Wl,-rpath,${BRIDGE_FASTCDR_LIB_DIR}"
)

add_library(bridge_core STATIC
  src/bridge/core/bridge_runner.cpp
)
target_link_libraries(bridge_core PUBLIC bridge_project_options)

add_library(bridge_dds_codecs STATIC
  src/bridge/dds/dds_translation.cpp
)
target_link_libraries(bridge_dds_codecs PUBLIC bridge_project_options bridge_core)

add_library(bridge_dds_runtime STATIC
  src/bridge/dds/dds_runtime.cpp
  src/bridge/dds/dds_factory.cpp
  src/bridge/dds/dds_command_ingress.cpp
  src/bridge/dds/dds_state_egress.cpp
)
target_link_libraries(bridge_dds_runtime PUBLIC
  bridge_project_options
  bridge_core
  bridge_config
  bridge_dds_codecs
  fastdds
  fastcdr
)

add_library(bridge_dds_generated STATIC
  idl/bridge/BridgeMessagesPubSubTypes.cxx
  idl/bridge/BridgeMessagesTypeObjectSupport.cxx
)
target_link_libraries(bridge_dds_generated PUBLIC
  bridge_project_options
  fastdds
  fastcdr
)

add_library(bridge_config STATIC
  src/bridge/config/bridge_config.cpp
  src/bridge/config/joint_map.cpp
)
target_link_libraries(bridge_config PUBLIC bridge_project_options bridge_core)

add_library(bridge_adapter_kepler_k2 STATIC
  src/bridge/adapters/kepler_k2/kepler_k2_adapter.cpp
)
target_link_libraries(bridge_adapter_kepler_k2 PUBLIC bridge_project_options)

add_library(bridge_extensions STATIC
  src/bridge/extensions/legacy_kepler_hooks.cpp
)
target_link_libraries(bridge_extensions PUBLIC bridge_project_options bridge_core)

add_library(bridge_app_support STATIC
  src/bridge/app/robot_assembly_factory.cpp
)
target_link_libraries(bridge_app_support PUBLIC
  bridge_project_options
  bridge_core
  bridge_dds_codecs
  bridge_dds_runtime
  bridge_dds_generated
  bridge_config
  bridge_adapter_kepler_k2
)

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
  BUILD_RPATH "${BRIDGE_FASTDDS_LIB_DIR};${BRIDGE_FASTCDR_LIB_DIR}"
)

if(BUILD_TESTING)
  function(bridge_add_test target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} PRIVATE ${ARGN})
    set_target_properties(${target_name} PROPERTIES
      BUILD_RPATH "${BRIDGE_FASTDDS_LIB_DIR};${BRIDGE_FASTCDR_LIB_DIR}"
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
    bridge_project_options
    bridge_core
    bridge_config
    bridge_dds_codecs
    bridge_dds_runtime
    bridge_dds_generated
  )

  bridge_add_test(generated_header_location_test
    tests/dds/generated_header_location_test.cpp
    bridge_project_options
  )

  bridge_add_test(dds_translation_test
    tests/dds/dds_translation_test.cpp
    bridge_project_options
    bridge_config
    bridge_dds_codecs
  )

  bridge_add_test(kepler_k2_adapter_test
    tests/adapters/kepler_k2_adapter_test.cpp
    bridge_project_options
    bridge_adapter_kepler_k2
  )

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
