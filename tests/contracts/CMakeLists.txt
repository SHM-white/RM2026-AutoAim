cmake_minimum_required(VERSION 3.16)
project(vision_contract_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(Eigen3 REQUIRED)

enable_testing()

add_executable(armor_geometry_contract_test ../armor_geometry_contract_test.cpp)
target_include_directories(armor_geometry_contract_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../.. ${OpenCV_INCLUDE_DIRS})
target_link_libraries(armor_geometry_contract_test PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)
add_test(NAME armor_geometry_contract COMMAND armor_geometry_contract_test)

add_executable(nav_protocol_contract_test ../nav_protocol_contract_test.cpp ../../tools/crc.cpp)
target_include_directories(nav_protocol_contract_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../..)
add_test(NAME nav_protocol_contract COMMAND nav_protocol_contract_test)
