
cmake_minimum_required(VERSION 3.20)

project(VehicleControl LANGUAGES CXX)

# Require standard C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable compiler warnings.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
elseif(MSVC)
    add_compile_options(/W4)
endif()

# Build the reusable vehicle control library.
add_library(vehicle_control STATIC
    lqr_controller.cpp
    steering_actuator.cpp
)

target_include_directories(vehicle_control PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# LQR controller tests.
add_executable(test_lqr_controller
    test_lqr_controller.cpp
)

target_link_libraries(test_lqr_controller PRIVATE
    vehicle_control
)

# Steering actuator tests.
add_executable(test_steering_actuator
    test_steering_actuator.cpp
)

target_link_libraries(test_steering_actuator PRIVATE
    vehicle_control
)

# Integrated control pipeline tests.
add_executable(test_control_pipeline
    test_control_pipeline.cpp
)

target_link_libraries(test_control_pipeline PRIVATE
    vehicle_control
)

# Register tests with CTest.
enable_testing()

add_test(NAME LQR_Controller
    COMMAND test_lqr_controller
)

add_test(NAME Steering_Actuator
    COMMAND test_steering_actuator
)

add_test(NAME Control_Pipeline
    COMMAND test_control_pipeline
)
