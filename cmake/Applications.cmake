add_library(gamepad_demo_support STATIC
  examples/gamepad/unitree_g1_gamepad.cpp
  examples/gamepad/gamepad_table.cpp
)
target_link_libraries(gamepad_demo_support PUBLIC
  bridge_project_options
  bridge_gamepad
)
if(TARGET bridge_gamepad_sdl)
  target_link_libraries(gamepad_demo_support PUBLIC bridge_gamepad_sdl)
endif()

add_executable(common_gamepad_wireless_codec_demo
  examples/gamepad/common_gamepad_wireless_codec_demo.cpp
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
    examples/unitree_g1/lowstate_frequency_receiver.cpp
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
    examples/unitree_g1/lowstate_field_recorder.cpp
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
    examples/gamepad/gamepad_dds_receiver.cpp
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
      examples/gamepad/gamepad_dds_sender.cpp
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
