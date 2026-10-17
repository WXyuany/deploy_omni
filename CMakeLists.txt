cmake_minimum_required(VERSION 3.16)
project(deploy_omni LANGUAGES CXX)

# Vendor binary SDKs target Linux; use a Linux host or container on macOS.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "deploy_omni requires Linux (x86_64 or aarch64). Use a Linux container on macOS.")
endif()

include(cmake/Options.cmake)
include(cmake/Dependencies.cmake)
include(cmake/Core.cmake)
include(cmake/Adapters.cmake)
include(cmake/Applications.cmake)
include(cmake/Tests.cmake)
