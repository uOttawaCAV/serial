# Set the minimum CMake version
cmake_minimum_required(VERSION 3.10)

# Set the project name to SerialComm
project(SerialComm)

# Set the C++ standard to C++11 (or change it based on your needs)
set(CMAKE_CXX_STANDARD 11)

# Include the 'include' directory where your header files are located
include_directories(include)

# Add the main library
add_library(SerialComm
    src/SerialComm.cpp   # The main source file
)

# Get all the example files in the 'examples' directory
file(GLOB EXAMPLES "examples/*.cpp")

# Create an executable for each example file
foreach(EXAMPLE_FILE ${EXAMPLES})
    # Extract the filename without extension for the target name
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)

    # Create an executable for each example and link it with the SerialComm library
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
    target_link_libraries(${EXAMPLE_NAME} SerialComm)

    # Set the output directory for all binaries to the 'bin' directory at the root of the project
    set_target_properties(${EXAMPLE_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ../bin/
    )
endforeach()

# Optionally, add tests
# add_executable(test_serialcomm tests/test_serialcomm.cpp)
# target_link_libraries(test_serialcomm SerialComm)
