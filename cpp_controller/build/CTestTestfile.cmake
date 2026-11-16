# CMake generated Testfile for 
# Source directory: C:/vehicle-dynamics-controls/cpp_controller
# Build directory: C:/vehicle-dynamics-controls/cpp_controller/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test("LQR_Controller" "C:/vehicle-dynamics-controls/cpp_controller/build/test_lqr_controller.exe")
set_tests_properties("LQR_Controller" PROPERTIES  _CMAKE_TEST_BUILD_DEPENDS "C:/vehicle-dynamics-controls/cpp_controller/build/test_lqr_controller.exe" _BACKTRACE_TRIPLES "C:/vehicle-dynamics-controls/cpp_controller/CMakeLists.txt;58;add_test;C:/vehicle-dynamics-controls/cpp_controller/CMakeLists.txt;0;")
add_test("Steering_Actuator" "C:/vehicle-dynamics-controls/cpp_controller/build/test_steering_actuator.exe")
set_tests_properties("Steering_Actuator" PROPERTIES  _CMAKE_TEST_BUILD_DEPENDS "C:/vehicle-dynamics-controls/cpp_controller/build/test_steering_actuator.exe" _BACKTRACE_TRIPLES "C:/vehicle-dynamics-controls/cpp_controller/CMakeLists.txt;62;add_test;C:/vehicle-dynamics-controls/cpp_controller/CMakeLists.txt;0;")
add_test("Control_Pipeline" "C:/vehicle-dynamics-controls/cpp_controller/build/test_control_pipeline.exe")
set_tests_properties("Control_Pipeline" PROPERTIES  _CMAKE_TEST_BUILD_DEPENDS "C:/vehicle-dynamics-controls/cpp_controller/build/test_control_pipeline.exe" _BACKTRACE_TRIPLES "C:/vehicle-dynamics-controls/cpp_controller/CMakeLists.txt;66;add_test;C:/vehicle-dynamics-controls/cpp_controller/CMakeLists.txt;0;")
