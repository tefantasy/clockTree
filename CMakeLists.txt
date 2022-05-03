cmake_minimum_required(VERSION 3.8.0)

project(cts)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")

file(GLOB SOURCES "src/*.cpp")

# flute sources
file(GLOB FLUTE_SOURCES "thirdparty/flute-3.1/*.c")
list(FILTER FLUTE_SOURCES EXCLUDE REGEX ".*rand-pts.c")
list(FILTER FLUTE_SOURCES EXCLUDE REGEX ".*flute-net.c")
list(FILTER FLUTE_SOURCES EXCLUDE REGEX ".*flute-ckt.c")

add_executable(cts ${SOURCES} ${FLUTE_SOURCES})

target_include_directories(cts PRIVATE "src")
target_include_directories(cts PRIVATE "include") # header only libraries
target_include_directories(cts PRIVATE "thirdparty/lemon/include") # lemon
target_include_directories(cts PRIVATE "thirdparty/flute-3.1") # flute

target_link_libraries(cts PRIVATE "${CMAKE_SOURCE_DIR}/thirdparty/lemon/lib/libemon.a") # lemon

# copy flute LUT data to bin directory
file(GLOB FLUTE_DATA "thirdparty/flute-3.1/*.dat")
file(COPY ${FLUTE_DATA} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
