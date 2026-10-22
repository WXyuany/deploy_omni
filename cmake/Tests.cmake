if(BUILD_TESTING AND BUILD_BRIDGE_DDS)
  # Each build owns its fixtures and generated test files; parallel robot builds
  # must not write temporary YAML into the source tree or share working files.
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/test-work")
  file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/config/" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/config")
  function(bridge_add_test target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} PRIVATE ${ARGN})
    set_target_properties(${target_name} PROPERTIES
      BUILD_RPATH "${BRIDGE_CYCLONE_LIB_DIR}"
    )
    add_test(NAME ${target_name} COMMAND ${target_name})
    set_tests_properties(${target_name} PROPERTIES
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/test-work")
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

if(BUILD_TESTING)
  find_package(Python3 QUIET COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    add_test(NAME cvte_carm_examples_test
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/demo/cvte_carm_examples_test.py")
    add_test(NAME verify_robot_test
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/demo/verify_robot_test.py")
  endif()
endif()
