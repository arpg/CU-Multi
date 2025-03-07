cmake_minimum_required(VERSION 3.5)
project(visualize_pc)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find PCL package
find_package(PCL 1.12 REQUIRED COMPONENTS common io visualization)

# Find VTK package
find_package(VTK 9.1 REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    RenderingCore
    RenderingOpenGL2
    CommonMath
)

# Include directories for PCL and VTK
include_directories(${PCL_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${VTK_LIBRARY_DIRS})

# Add compile definitions for PCL and VTK
add_definitions(${PCL_DEFINITIONS} ${VTK_DEFINITIONS})

# Define the path to the config directory
add_definitions(-DCAMERA_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/camera_params.txt")
add_definitions(-DDATASET_PATH="${CMAKE_SOURCE_DIR}/coloradar_plus_dataset/")

# Add the executable
add_executable(visualize_pc src/visualize_pc.cpp)

# Link PCL and VTK libraries
target_link_libraries(visualize_pc ${PCL_LIBRARIES} ${VTK_LIBRARIES})
