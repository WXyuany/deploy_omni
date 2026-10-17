add_library(bridge_gamepad STATIC
  src/bridge/gamepad/common_gamepad.cpp
)
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
