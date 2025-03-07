cmake_minimum_required(VERSION 3.5)
project(cu_multi_vis_toolkit)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Ensure the system is macOS
if(APPLE)
    message(STATUS "Building on macOS")
endif()

# Find PCL package
# find_package(PCL 1.12 REQUIRED COMPONENTS common io visualization filters)
find_package(PCL REQUIRED COMPONENTS common io visualization filters)

# # Find VTK package
# find_package(VTK 9.1 REQUIRED COMPONENTS
#     CommonCore
#     CommonDataModel
#     RenderingCore
#     RenderingOpenGL2
#     CommonMath
# )
# Find VTK package
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    RenderingCore
    RenderingOpenGL2
    CommonMath
)

# Include directories for PCL, VTK, and project headers
include_directories(${PCL_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

# Link directories for PCL and VTK
link_directories(${PCL_LIBRARY_DIRS} ${VTK_LIBRARY_DIRS})

# Add compile definitions for PCL and VTK
add_definitions(${PCL_DEFINITIONS} ${VTK_DEFINITIONS})

# Define the path to the config directory
add_definitions(-DCAMERA_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/camera_params.txt")
add_definitions(-DMESH_DIR_PATH="${CMAKE_SOURCE_DIR}/mesh/")
add_definitions(-DDATASET_PATH="/media/donceykong/donceys_data_ssd/datasets/CU_MULTI/data/kittredge_loop/robot1/")

# Collect all source files in src
file(GLOB_RECURSE PROJECT_SOURCES "src/*.cpp")

# Create the library with the source files
add_library(cu_multi_vis_toolkit ${PROJECT_SOURCES})

# Link PCL and VTK libraries to cu_multi_vis_toolkit
target_link_libraries(cu_multi_vis_toolkit ${PCL_LIBRARIES} ${VTK_LIBRARIES})

# Add the test executable
add_executable(test_visualization src/test_visualization.cpp)

# Link the test executable to cu_multi_vis_toolkit, PCL, and VTK
target_link_libraries(test_visualization cu_multi_vis_toolkit ${PCL_LIBRARIES} ${VTK_LIBRARIES})

# # macOS-specific adjustments
# if(APPLE)
#     # Ensure the use of OpenGL framework if needed
#     target_link_libraries(test_visualization "-framework OpenGL")

#     # Workaround for macOS library linking if issues arise
#     set(CMAKE_MACOSX_RPATH 1) # Enable RPATH handling
# endif()
