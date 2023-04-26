cmake_minimum_required (VERSION 3.16)
cmake_policy (VERSION 3.16)
project (Hello, CXX)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(drake CONFIG REQUIRED PATHS /opt/drake)

set (list 2 3 7)
set (list "h;i")
foreach (n ${list})
  message ("Element: ${n}")
endforeach ()
message (${list})
message (${CMAKE_VERSION})
message (${drake_DIR})
add_executable (Hello h.cpp hh.cpp)

