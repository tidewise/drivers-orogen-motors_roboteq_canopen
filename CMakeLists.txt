PROJECT(motors_roboteq_canopen)
cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_STANDARD 17)
SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.orogen/config")
INCLUDE(motors_roboteq_canopenBase)

if (ROCK_TEST_ENABLED)
    enable_testing()
    find_package(Rock REQUIRED)
    find_package(Syskit REQUIRED)
    add_subdirectory(test)
    syskit_orogen_tests(test)
endif()

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

