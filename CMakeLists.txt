cmake_minimum_required(VERSION 3.10)
project(FlightPlanner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 COMPONENTS Widgets REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(flight-planner src/main.cpp)
target_link_libraries(flight-planner PRIVATE Qt6::Widgets)