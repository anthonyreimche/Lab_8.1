cmake_minimum_required(VERSION 3.30)
project(Lab_8_new)

set(CMAKE_CXX_STANDARD 14)

# Add Windows socket library
if(WIN32)
    link_libraries(wsock32)
endif()

# Add warning disables for MSVC
if(MSVC)
    add_compile_options(/wd4996)
else()
    add_compile_options(-Wno-deprecated)
endif()

add_executable(Lab_8_new scara_main.cpp robot.cpp scara.cpp scaraConsole.cpp)
