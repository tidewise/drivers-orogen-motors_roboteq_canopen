rock_gtest(test_suite suite.cpp
    test_Helpers.cpp
    SOURCES
        ../tasks/Helpers.cpp
    HEADERS
        ../tasks/Helpers.hpp
    DEPS_PKGCONFIG
        raw_io
)

target_link_libraries(test_suite ${MOTORS_ROBOTEQ_CANOPEN_TASKLIB_DEPENDENT_LIBRARIES})