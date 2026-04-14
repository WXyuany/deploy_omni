cmake_minimum_required(VERSION 3.16)

project(adapter_any_robot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_library(bridge_project_options INTERFACE)
target_compile_features(bridge_project_options INTERFACE cxx_std_20)
target_include_directories(bridge_project_options INTERFACE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_library(bridge_core STATIC
  src/bridge/core/bridge_runner.cpp
)
target_link_libraries(bridge_core PUBLIC bridge_project_options)

add_library(bridge_dds_codecs STATIC
  src/bridge/dds/lowcmd_codec.cpp
  src/bridge/dds/lowstate_codec.cpp
)
target_link_libraries(bridge_dds_codecs PUBLIC bridge_project_options bridge_core)

add_library(bridge_config STATIC
  src/bridge/config/bridge_config.cpp
  src/bridge/config/joint_map.cpp
)
target_link_libraries(bridge_config PUBLIC bridge_project_options bridge_core)

add_library(bridge_adapter_kepler_k2 STATIC
  src/bridge/adapters/kepler_k2/kepler_k2_adapter.cpp
)
target_link_libraries(bridge_adapter_kepler_k2 PUBLIC bridge_project_options)

add_library(bridge_adapter_pm01 STATIC
  src/bridge/adapters/pm01/pm01_adapter.cpp
)
target_link_libraries(bridge_adapter_pm01 PUBLIC bridge_project_options)

add_library(bridge_extensions STATIC
  src/bridge/extensions/legacy_kepler_hooks.cpp
)
target_link_libraries(bridge_extensions PUBLIC bridge_project_options bridge_core)

add_executable(kepler_k2_bridge
  src/apps/kepler_k2_bridge_main.cpp
)
target_link_libraries(kepler_k2_bridge PRIVATE
  bridge_project_options
  bridge_core
  bridge_dds_codecs
  bridge_config
  bridge_adapter_kepler_k2
)

if(BUILD_TESTING)
  function(bridge_add_test target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} PRIVATE ${ARGN})
    add_test(NAME ${target_name} COMMAND ${target_name})
  endfunction()

  bridge_add_test(bridge_runner_test
    tests/core/bridge_runner_test.cpp
    bridge_project_options
    bridge_core
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
    bridge_dds_codecs
  )

  bridge_add_test(kepler_k2_adapter_test
    tests/adapters/kepler_k2_adapter_test.cpp
    bridge_project_options
    bridge_adapter_kepler_k2
  )

  bridge_add_test(pm01_adapter_test
    tests/adapters/pm01_adapter_test.cpp
    bridge_project_options
    bridge_adapter_pm01
  )

  bridge_add_test(hooks_isolation_test
    tests/extensions/hooks_isolation_test.cpp
    bridge_project_options
    bridge_extensions
  )
endif()
