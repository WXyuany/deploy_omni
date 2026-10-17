set(BRIDGE_ROBOT "" CACHE STRING
  "Robot adapter to build: kepler_k2, pm01, unitree_g1, cvte_carm, or all")
set_property(CACHE BRIDGE_ROBOT PROPERTY STRINGS
  kepler_k2 pm01 unitree_g1 cvte_carm all legacy)

# Keep the per-adapter switches for older build scripts. New builds should use
# BRIDGE_ROBOT so an SDK unavailable on the target machine is never selected.
option(BUILD_ROBOT_KEPLER_K2 "Build kepler_k2 adapter support" OFF)
option(BUILD_ROBOT_PM01 "Build pm01 adapter support" OFF)
option(BUILD_ROBOT_UNITREE_G1 "Build unitree_g1 adapter support" OFF)
option(BUILD_ROBOT_CVTE_CARM "Build CVTE CArm adapter support" OFF)
option(BUILD_GAMEPAD_SDL "Build SDL2 gamepad input support" ON)
option(BUILD_UNITREE_G1_HARDWARE_TESTS "Build Unitree G1 hardware interface checks" OFF)
option(BUILD_UNITREE_G1_MOTION_DEMO
  "Build the interactive Unitree G1 high-level motion demo" OFF)

if("${BRIDGE_ROBOT}" STREQUAL "")
  if(BUILD_ROBOT_KEPLER_K2 OR BUILD_ROBOT_PM01 OR BUILD_ROBOT_UNITREE_G1 OR
     BUILD_ROBOT_CVTE_CARM)
    set(BRIDGE_ROBOT "legacy")
    message(WARNING
      "BUILD_ROBOT_* switches are deprecated; use BRIDGE_ROBOT for new build configurations.")
  else()
    message(FATAL_ERROR
      "BRIDGE_ROBOT is required. Choose one of: kepler_k2, pm01, unitree_g1, cvte_carm, all. "
      "Use BRIDGE_ROBOT=legacy only for the deprecated BUILD_ROBOT_* switches.")
  endif()
elseif("${BRIDGE_ROBOT}" STREQUAL "legacy")
  message(WARNING
    "BRIDGE_ROBOT=legacy is deprecated; use one explicit robot selection instead.")
elseif("${BRIDGE_ROBOT}" STREQUAL "all")
  set(BUILD_ROBOT_KEPLER_K2 ON)
  set(BUILD_ROBOT_PM01 ON)
  set(BUILD_ROBOT_UNITREE_G1 ON)
  # CVTE has a separate binary SDK and is enabled explicitly to avoid making
  # the aggregate build depend on a machine-local vendor package.
  set(BUILD_ROBOT_CVTE_CARM OFF)
elseif("${BRIDGE_ROBOT}" STREQUAL "kepler_k2")
  set(BUILD_ROBOT_KEPLER_K2 ON)
  set(BUILD_ROBOT_PM01 OFF)
  set(BUILD_ROBOT_UNITREE_G1 OFF)
  set(BUILD_ROBOT_CVTE_CARM OFF)
elseif("${BRIDGE_ROBOT}" STREQUAL "pm01")
  set(BUILD_ROBOT_KEPLER_K2 OFF)
  set(BUILD_ROBOT_PM01 ON)
  set(BUILD_ROBOT_UNITREE_G1 OFF)
  set(BUILD_ROBOT_CVTE_CARM OFF)
elseif("${BRIDGE_ROBOT}" STREQUAL "unitree_g1")
  set(BUILD_ROBOT_KEPLER_K2 OFF)
  set(BUILD_ROBOT_PM01 OFF)
  set(BUILD_ROBOT_UNITREE_G1 ON)
  set(BUILD_ROBOT_CVTE_CARM OFF)
elseif("${BRIDGE_ROBOT}" STREQUAL "cvte_carm")
  set(BUILD_ROBOT_KEPLER_K2 OFF)
  set(BUILD_ROBOT_PM01 OFF)
  set(BUILD_ROBOT_UNITREE_G1 OFF)
  set(BUILD_ROBOT_CVTE_CARM ON)
else()
  message(FATAL_ERROR
    "Unsupported BRIDGE_ROBOT='${BRIDGE_ROBOT}'. Choose one of: "
    "kepler_k2, pm01, unitree_g1, cvte_carm, all.")
endif()

if("${BRIDGE_ROBOT}" STREQUAL "cvte_carm")
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

if(NOT BUILD_ROBOT_KEPLER_K2 AND NOT BUILD_ROBOT_PM01 AND NOT BUILD_ROBOT_UNITREE_G1 AND NOT BUILD_ROBOT_CVTE_CARM)
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

set(BRIDGE_BUILD_DDS 0)
if(BUILD_BRIDGE_DDS)
  set(BRIDGE_BUILD_DDS 1)
endif()
