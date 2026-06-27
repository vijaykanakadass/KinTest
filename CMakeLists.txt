cmake_minimum_required(VERSION 3.24)
project(kinplay LANGUAGES CXX)

# Use modern URL-download timestamp behavior (silences CMP0135 warning).
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

option(KINPLAY_BUILD_TESTS "Build kinplay tests" ON)

include(FetchContent)

# Eigen 3.4.0 — fetched as a tarball from a GitHub mirror because gitlab.com
# is unreachable and `git clone` is blocked by the build environment proxy
# (only HTTPS tarball downloads work). Hash pinned for reproducibility.
FetchContent_Declare(
  Eigen3
  URL      https://codeload.github.com/eigen-mirror/eigen/tar.gz/refs/tags/3.4.0
  URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen3)

# nlohmann/json 3.11.3 — tarball fetch (git clone is proxy-blocked). Used only
# by the io layer for mechanism definition files; core stays json-free.
FetchContent_Declare(
  nlohmann_json
  URL      https://codeload.github.com/nlohmann/json/tar.gz/refs/tags/v3.11.3
  URL_HASH SHA256=0d8ef5af7f9794e3263480193c491549b2ba6cc74bb018906202ada498a79406
)
set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(nlohmann_json)

add_library(kinplay_core STATIC
  src/core/joint.cpp
  src/core/mechanism.cpp
  src/core/jacobian.cpp
  src/core/ik/dls.cpp
)
target_include_directories(kinplay_core PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(kinplay_core PUBLIC Eigen3::Eigen)
target_compile_options(kinplay_core PRIVATE -Wall -Wextra -Wpedantic)

# io layer: mechanism JSON load/save + built-in presets. Depends on core and
# nlohmann/json. Kept separate so core has no JSON dependency (§4).
add_library(kinplay_io STATIC
  src/io/mechanism_json.cpp
  src/io/presets.cpp
)
target_link_libraries(kinplay_io PUBLIC kinplay_core PRIVATE nlohmann_json::nlohmann_json)
target_compile_options(kinplay_io PRIVATE -Wall -Wextra -Wpedantic)

add_executable(kinplay src/main.cpp)
target_link_libraries(kinplay PRIVATE kinplay_core kinplay_io)

if(KINPLAY_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
