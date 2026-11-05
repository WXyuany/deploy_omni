execute_process(
  COMMAND "$ENV{BRIDGE_AR}" t "$ENV{BRIDGE_ARCHIVE}"
  OUTPUT_VARIABLE members
  RESULT_VARIABLE list_result)
if(NOT list_result EQUAL 0)
  message(FATAL_ERROR "cannot inspect archive")
endif()
if(members MATCHES "(^|\\n)service_main\\.cpp\\.o(/|\\n|$)")
  execute_process(
    COMMAND "$ENV{BRIDGE_AR}" d "$ENV{BRIDGE_ARCHIVE}" service_main.cpp.o
    RESULT_VARIABLE delete_result)
  if(NOT delete_result EQUAL 0)
    message(FATAL_ERROR "cannot remove service_main.cpp.o")
  endif()
endif()
