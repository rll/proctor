cmake_minimum_required(VERSION 2.6)

# PCL has modules that find Flann and Eigen
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} /home/pabbeel/wh/pcl-trunk/build /home/pabbeel/wh/pcl-trunk/cmake/Modules)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(PCL 1.0 REQUIRED COMPONENTS common features filters io visualization)
include_directories(${PCL_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(VTK 5.8 REQUIRED)
include(${VTK_USE_FILE})

add_executable(main src/main.cpp src/detector.cpp src/proctor.cpp src/scanner.cpp)
target_link_libraries(main ${FLANN_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} vtkCommon vtkFiltering vtkGraphics vtkIO)
