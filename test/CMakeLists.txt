rock_gtest(test_suite suite.cpp
    test_Helpers.cpp
    SOURCES
        ../tasks/Helpers.cpp
    HEADERS
        ../tasks/Helpers.hpp
    DEPS_PKGCONFIG
        raw_io
)
