cmake_minimum_required (VERSION 2.8.11)
project (ppcvivace)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


add_library(libppcvivace "")
set_property(TARGET libppcvivace PROPERTY CXX_STANDARD 17)
target_include_directories (libppcvivace PUBLIC ${PROJECT_SOURCE_DIR}/include)

add_subdirectory (src)
add_subdirectory (tests)

