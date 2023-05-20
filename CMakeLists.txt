cmake_minimum_required (VERSION 3.16)
cmake_policy (VERSION 3.16)
project (Hello, CXX)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)
message ("CMake version: ${CMAKE_VERSION}")

set (DEFAULT_COMPILE_OPTIONS -Wall -Wextra -Werror)
#add_compile_options(-Wall -Wextra -pedantic -Werror) Do not use.
message("Compile flags: ")
foreach (c ${DEFAULT_COMPILE_OPTIONS})
  message(${c})
endforeach()

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "In-source builds are not allowed.")
endif()


find_package(drake CONFIG REQUIRED PATHS /opt/drake)
find_package(gflags REQUIRED)

message ("Drake found at: ${drake_DIR}")

add_subdirectory(src)

