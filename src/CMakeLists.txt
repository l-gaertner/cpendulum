add_executable(main main/main.cpp)
add_library(main_lib STATIC
    main/Pendulum.cpp
    main/Weight.cpp
)
target_link_libraries(main PRIVATE main_lib)

find_package(Curses REQUIRED)
target_include_directories(main PRIVATE ${CURSES_INCLUDE_DIR})
target_link_libraries(main PRIVATE ${CURSES_LIBRARIES})

# BUILD_TESTING variable is created by include(CTest)
# It is set to ON by default
if (BUILD_TESTING)

    add_executable(my_test
        test/InitialStatePendulumTest.cpp
        test/SimulationPendulumTest.cpp
    )

    target_link_libraries(my_test PRIVATE GTest::gtest_main)
    target_link_libraries(my_test PRIVATE main_lib)

    include(GoogleTest)
    # Finds all the Google tests associated with the executable
    gtest_discover_tests(my_test)
endif()
