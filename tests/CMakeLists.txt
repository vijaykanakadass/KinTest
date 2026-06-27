include(FetchContent)

# Tarball fetch instead of git clone — see top-level CMakeLists.txt.
FetchContent_Declare(
  googletest
  URL      https://codeload.github.com/google/googletest/tar.gz/refs/tags/v1.14.0
  URL_HASH SHA256=8ad598c73ad796e0d8280b082cebd82a630d73e73cd3c70057938a6501bba5d7
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

add_executable(test_fk test_fk.cpp)
target_link_libraries(test_fk PRIVATE kinplay_core GTest::gtest_main)
target_compile_options(test_fk PRIVATE -Wall -Wextra -Wpedantic)
gtest_discover_tests(test_fk)

add_executable(test_jacobian test_jacobian.cpp)
target_link_libraries(test_jacobian PRIVATE kinplay_core GTest::gtest_main)
target_compile_options(test_jacobian PRIVATE -Wall -Wextra -Wpedantic)
gtest_discover_tests(test_jacobian)

add_executable(test_ik_dls test_ik_dls.cpp)
target_link_libraries(test_ik_dls PRIVATE kinplay_core GTest::gtest_main)
target_compile_options(test_ik_dls PRIVATE -Wall -Wextra -Wpedantic)
gtest_discover_tests(test_ik_dls)

add_executable(test_dh test_dh.cpp)
target_link_libraries(test_dh PRIVATE kinplay_core GTest::gtest_main)
target_compile_options(test_dh PRIVATE -Wall -Wextra -Wpedantic)
gtest_discover_tests(test_dh)

add_executable(test_ik_ccd test_ik_ccd.cpp)
target_link_libraries(test_ik_ccd PRIVATE kinplay_core GTest::gtest_main)
target_compile_options(test_ik_ccd PRIVATE -Wall -Wextra -Wpedantic)
gtest_discover_tests(test_ik_ccd)

add_executable(test_io test_io.cpp)
target_link_libraries(test_io PRIVATE kinplay_io GTest::gtest_main)
target_compile_options(test_io PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(test_io PRIVATE
  KINPLAY_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")
gtest_discover_tests(test_io)
