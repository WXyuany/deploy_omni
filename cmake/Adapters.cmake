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
      examples/unitree_g1/unitree_g1_motion_demo.cpp
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

  add_executable(unitree_g1_sensor_reader examples/unitree_g1/unitree_g1_sensor_reader.cpp)
  target_link_libraries(unitree_g1_sensor_reader PRIVATE bridge_adapter_unitree_g1)
  add_dependencies(unitree_g1_sensor_reader unitree_sdk2_nomain)
  if(BUILD_TESTING)
    add_test(NAME unitree_g1_sensor_reader_help COMMAND unitree_g1_sensor_reader --help)
    add_test(NAME unitree_g1_sensor_reader_invalid_args
      COMMAND unitree_g1_sensor_reader --duration invalid)
    set_tests_properties(unitree_g1_sensor_reader_invalid_args PROPERTIES WILL_FAIL TRUE)
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
  # The vendor library has sibling transitive dependencies (jsoncpp, mlog).
  # DT_RPATH lets the executable locate them without a global LD_LIBRARY_PATH.
  target_link_options(bridge_adapter_cvte_carm INTERFACE "-Wl,--disable-new-dtags")
  target_link_directories(bridge_adapter_cvte_carm PUBLIC
    "${BRIDGE_CVTE_SDK_DIR}/lib"
  )
  set_target_properties(bridge_adapter_cvte_carm PROPERTIES
    BUILD_RPATH "${BRIDGE_CVTE_SDK_DIR}/lib"
    INSTALL_RPATH "${BRIDGE_CVTE_SDK_DIR}/lib"
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
