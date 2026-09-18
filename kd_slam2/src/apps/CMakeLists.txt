include_directories(.)

add_subdirectory(nodes)

if (BUILD_TESTS)
   add_subdirectory(tests)
endif()
